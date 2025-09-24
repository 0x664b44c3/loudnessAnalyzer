/*
 *  Rec. ITU-R BS.1770-2 / EBU R 128-2011 DSP module
 *
 * BE GAY  🟥🟧🟨🟩🟦🟪
 * 🟥🟧🟨🟩🟦🟪 DO UTF8
**/

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_task_wdt.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <driver/gpio.h>
#include <driver/i2s_std.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <string.h>
#include "dsp.h"

#include "../managed_components/espressif__esp-dsp/modules/common/include/esp_dsp.h"


//uncomment to use a gpio to measure actual processing times
#define TIMING_PROBE 5

//max channels (only stereo supported down the chain atm)
#define MAX_CHANNEL 2
#define N_BUFFERS MAX_CHANNEL + 1

#define NOMINAL_SR 48000


//48 coefficients as of itu rec, will be used to calculate int16/Q15 factors during init
static float ituUpsamplerCoeff[] = {
    /* phase 0        phase 1            phase 2           phase 3      */
    0.0017089843750, -0.0291748046875, -0.0189208984375, -0.0083007812500,
    0.0109863281250,  0.0292968750000,  0.0330810546875,  0.0148925781250,
    -0.0196533203125, -0.0517578125000, -0.0582275390625, -0.0266113281250,
    0.0332031250000,  0.0891113281250,  0.1015625000000,  0.0476074218750,
    -0.0594482421875, -0.1665039062500, -0.2003173828125, -0.1022949218750,
    0.1373291015625,  0.4650878906250,  0.7797851562500,  0.9721679687500,
    0.9721679687500,  0.7797851562500,  0.4650878906250,  0.1373291015625,
    -0.1022949218750, -0.2003173828125, -0.1665039062500, -0.0594482421875,
    0.0476074218750,  0.1015625000000,  0.0891113281250,  0.0332031250000,
    -0.0266113281250, -0.0582275390625, -0.0517578125000, -0.0196533203125,
    0.0148925781250,  0.0330810546875,  0.0292968750000,  0.0109863281250,
    -0.0083007812500, -0.0189208984375, -0.0291748046875,  0.0017089843750
};

volatile int digiPeak[2];
volatile int digiPeakHold[2];

volatile float LUFS_M=0;
volatile float LUFS_I=-99;
volatile float truePeak=0;
volatile float LUFS_S=0;
volatile float LUFS_avg_l=0;
volatile float LUFS_avg_r=0;
volatile float LUFS_peak=0;
volatile float LUFS_min=0;

volatile float chCorr=0;
#define chunkSize (NOMINAL_SR / 100)

float rtaLevels[31];
volatile float rms[2];

/** EBU R-128 weighting coefficients */
static const float coeffPreFilter[] = {
    /* b0 */  1.53512485958697,
    /* b1 */ -2.69169618940638,
    /* b2 */  1.19839281085285,
    /* a1 */ -1.69065929318241,
    /* a2 */  0.73248077421585
};
static const float coeffHiPass[] =     {
    /* b0 */  1.0,
    /* b1 */ -2.0,
    /* b2 */  1.0,
    /* a1 */ -1.99004745483398,
    /* a2 */  0.99007225036621
};


#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif


#define SCALE24(a) (1.0 / 2147483392.0 * a)
#ifndef CMAX
#define CMAX(a, b) ((a>b)?a:b)
#define CMIN(a, b) ((a<b)?a:b)
#endif

r128Ch_t r128Ch[MAX_CHANNEL];

#define RESULT_BUFFERS 8
#define INPUT_BUFFERS 4
//dsp results to go into the output queue (statically allocated)
dspResults_t dspResult[RESULT_BUFFERS];

SemaphoreHandle_t dspSemaphore;
static float * processingBuffer[N_BUFFERS];

static i2s_chan_handle_t rx_chan;

volatile int nBuff  = 0; //< used for stats

static int32_t *rec_buffer[INPUT_BUFFERS];

TaskHandle_t dspTask = NULL;
TaskHandle_t recTask = NULL;
QueueHandle_t inQueue;

rtaChannel_t rtaChannels[31];

//1min of short history
#define SHORT_HIST_LEN 600

/** level history, channel mapping
 *   0 - LUFS_l   (Lk)
 *   1 - LUFS_r   (Lk)
 *   2 - LUFS_mix (Lk)
**/
float lvlShortHistory[3][SHORT_HIST_LEN];
float lufLongHistory[300]; //10min //updated each 5 seconds with the 10s gated average

int silenceSegments=0;


static int currentRecBuffer = 0;
static int currentDspResult=0;

float correlationHist[40];
float tpkBlockHist[10];
float tpkHist[SHORT_HIST_LEN];
int chPkBlkHist[2][10];
int chPkHist[2][10];


static void IRAM_ATTR audio_rx_task(void*) {
    size_t r_bytes = 0;
    const int buffSize = chunkSize * 2 * sizeof(int32_t);
    vTaskCoreAffinitySet(NULL, 2);
    while(1)
    {
        int32_t * bp = rec_buffer[currentRecBuffer];
        uint8_t *r_buf = (uint8_t*)bp;
        if (i2s_channel_read(rx_chan, r_buf, buffSize, &r_bytes, 1000) == ESP_OK) {
            ++nBuff;
            if (r_bytes != buffSize) {
                printf("Warning: I2S read size mismatch\n");
            }
            currentRecBuffer++;
            if(currentRecBuffer == INPUT_BUFFERS)
                currentRecBuffer=0;

            if (dspTask)
                xQueueSend(inQueue, &bp, 0);

        } else {
            printf("Read Task: i2s read failed\n");
        }
        taskYIELD();
    }
}

static const float rtaBands[] = {   20,   25, 31.5,   40,   50,   63,   80,  100,
                                 125,  160,  200,  250,  315,  400,  500,  630,
                                 800, 1000, 1250, 1600, 2000, 2500, 3150, 4000,
                                 5000, 6300, 8000,10000,12500,16000,20000,    0};

static const float terz_Q = 4.3;

static int rtaCtr=0;
static float rtaCoeff[31][5];

static int16_t monoBuffer[2][chunkSize];
static int16_t upsampleBuffer[chunkSize*4];

#define ITU_POLYPHASE_TAPS 12

//buffer for the upsampler coefficients (time-memory tradeoff: store 12 permutations of the factors for each phase (2kBytes of coeffs)
int16_t polyphaseCoeff[12*4][16]; //only 12 coefficients but do alignment for optimization of interpolator
float   polyphaseCoeffF32[4][12][16]; //only 12 coefficients but do alignment for optimization of interpolator

typedef struct {
    int16_t delay[ITU_POLYPHASE_TAPS];
    int ptr;
}polyPhase16_t ;


typedef struct {
    float delay[ITU_POLYPHASE_TAPS];
    int ptr;
}polyPhaseF32_t ;



//two upamplers (one per channel)
polyPhase16_t upsamplers[2];
polyPhaseF32_t upsamplersF32[2];
static float upsampleBufferF[480*4];
static float tpkLPF[5];
static float tpk_w1[2];
static float tpk_w2[2];


void preloadInterpolatorCoeffs() {
    for (int perm = 0;perm<12;++perm)
    {
        for(int phase=0;phase<4;++phase)
        {
            int k0=perm;
            for(int i=0;i<12;++i)
            {
                float c0 = ituUpsamplerCoeff[phase+(k0++)*4];
                if(k0==12)
                    k0=0;
                int coeff = c0 * 32767.0;
                int block = (11-perm) * 4 + 3 - phase;
                polyphaseCoeffF32[3-phase][11-perm][i] = c0;
                polyphaseCoeff[block][i] = coeff&0xffff;
            }
        }
    }
    dsps_biquad_gen_lpf_f32(&tpkLPF, 0.125, 1);
}
void initInterpolatorCtx(polyPhase16_t*ppi, int N)
{
    for(int i=0;i<N;++i)
    {
        ppi[i].ptr = 0;
        for(int j=0;j<ITU_POLYPHASE_TAPS;++j)
            ppi->delay[j]=0;
    }
}

void IRAM_ATTR polyphase_upsampler_x4_i16(int16_t * in, int16_t * out, int inSize, polyPhase16_t *filter)
{
    int wp = filter->ptr;
    int16_t acc=0;
    for(int i=0;i<inSize;++i)
    {
        filter->delay[wp] = *in++;
        for(int j=0;j<4;++j)
        {
            dsps_dotprod_s16(filter->delay, polyphaseCoeff[wp*4+j], &acc, ITU_POLYPHASE_TAPS, 0);
            *out++=acc;
        }
        ++wp;
        if(wp==ITU_POLYPHASE_TAPS)
            wp=0;
    }
    filter->ptr = wp;
}


void IRAM_ATTR polyphase_upsampler_x4_f32(float * in, float * out, int inSize, polyPhaseF32_t *filter)
{
    int wp = filter->ptr;
    float acc=0;
    for(int i=0;i<inSize;++i)
    {
        filter->delay[wp] = *in++;
        dsps_dotprod_f32(filter->delay, polyphaseCoeffF32[0][wp], &acc, ITU_POLYPHASE_TAPS);
        *out++=acc;
        dsps_dotprod_f32(filter->delay, polyphaseCoeffF32[1][wp], &acc, ITU_POLYPHASE_TAPS);
        *out++=acc;
        dsps_dotprod_f32(filter->delay, polyphaseCoeffF32[2][wp], &acc, ITU_POLYPHASE_TAPS);
        *out++=acc;
        dsps_dotprod_f32(filter->delay, polyphaseCoeffF32[3][wp], &acc, ITU_POLYPHASE_TAPS);
        *out++=acc;
        ++wp;
        if(wp==ITU_POLYPHASE_TAPS)
            wp=0;
    }
    filter->ptr = wp;
}

float IRAM_ATTR calcTruePeakF32(float * ch0, float * ch1)
{
    float maxPeak=0;
    polyphase_upsampler_x4_f32(ch0, upsampleBufferF, chunkSize,  &upsamplersF32[0]);
    dsps_biquad_f32_ae32(upsampleBufferF, upsampleBufferF, 4 * chunkSize, tpkLPF, tpk_w1);
    for(int i=0;i<chunkSize*4;++i)
        maxPeak=CMAX(maxPeak, fabs(upsampleBufferF[i]));
    polyphase_upsampler_x4_f32(ch1, upsampleBufferF, chunkSize,  &upsamplersF32[1]);
    dsps_biquad_f32_ae32(upsampleBufferF, upsampleBufferF, 4 * chunkSize, tpkLPF, tpk_w2);
    for(int i=0;i<chunkSize*4;++i)
        maxPeak=CMAX(maxPeak, fabs(upsampleBufferF[i]));
    return maxPeak;
}

//split stereo buffer 32bit into 2 buffer of scaled 16bit
int IRAM_ATTR calcTruePeak(int32_t *inputBuffer, int16_t gainFactor, int N)
{
    for(int i=0;i<chunkSize;++i)
    {
         monoBuffer[0][i] = *inputBuffer++ / 262144;
         monoBuffer[1][i] = *inputBuffer++ / 262144;
    }
    // int16_t * id = (int16_t*)inputBuffer;
    // dsps_mulc_s16_ansi(&id[1], monoBuffer[0], N, 8192, 4, 1);
    // dsps_mulc_s16_ansi(&id[3], monoBuffer[1], N, 8192, 4, 1);
    int maxPeak=0;
    polyphase_upsampler_x4_i16(monoBuffer[0], upsampleBuffer, chunkSize,  &upsamplers[0]);
    for(int i=0;i<chunkSize*4;++i)
        maxPeak=CMAX(maxPeak, abs(upsampleBuffer[i]));
    polyphase_upsampler_x4_i16(monoBuffer[1], upsampleBuffer, chunkSize,  &upsamplers[1]);
    for(int i=0;i<chunkSize*4;++i)
        maxPeak=CMAX(maxPeak, abs(upsampleBuffer[i]));
    return maxPeak;
}

const float threshold_0 = -70.0;
float       threshold_1 = -70.0;

IRAM_ATTR void copyChannelData(int32_t * src, float ** channelBuffers, int N, int32_t * dPeak0, int32_t * dPeak1) {

    int32_t peak_0 = 0;
    int32_t peak_1 = 0;
    float *d0=channelBuffers[0];
    float *d1=channelBuffers[1];
    for(int i=0;i<N;++i)
    {
        int chv = *src++;
        peak_0 = CMAX(abs(chv), peak_0);
        *d0++ = SCALE24(chv);
        chv = *src++;
        peak_1 = CMAX(abs(chv), peak_1);
        *d1++ = SCALE24(chv);
    }
    if (dPeak0)
        *dPeak0 = peak_0;
    if (dPeak1)
        *dPeak1 = peak_1;
}



float accu=0;

void IRAM_ATTR calcRmsSegment(float ** processingBuffers, float gain, int nChannel, int segmentCtr)
{
    for(int ch=0;ch<nChannel;++ch)
    {
        r128Ch_t * ctx = &r128Ch[ch];

        float *buffer  = processingBuffers[ch];
        float *buffer2 = processingBuffers[nChannel];

        dsps_mulc_f32(buffer, buffer, chunkSize, gain, 1, 1);

        dsps_biquad_f32(buffer , buffer2, chunkSize, coeffPreFilter, ctx->filter1_z);
        dsps_biquad_f32(buffer2, buffer2, chunkSize, coeffHiPass, ctx->filter2_z);

        //weighted mean squares
        dsps_dotprode_f32(buffer2, buffer2, &accu, chunkSize, 1, 1);
        ctx->k_segments[segmentCtr] = accu / ((float) chunkSize);

        //perform unweighted mean squares (channel RMS)
        dsps_dotprode_f32(buffer, buffer, &accu, chunkSize, 1, 1);
        ctx->rms_segments[segmentCtr] = accu / ((float) chunkSize);

        float accu=0;
        int idx=segmentCtr;
        for(int n=0;n<25;++n)
        { //250ms history for fast RMS
            accu+=ctx->rms_segments[idx--];
            if(idx<0)
                idx=39;
        }
        ctx->fastRMS = accu/25.0;
    }
}

static void IRAM_ATTR calcRTA()
{
    for(int band=0;band<31;++band)
    {
        dsps_biquad_f32(processingBuffer[0], processingBuffer[2], chunkSize, rtaCoeff[band], rtaChannels[band].filter_w);
        dsps_dotprod_f32(processingBuffer[2], processingBuffer[2], &accu, chunkSize);
        rtaChannels[band].rta_segments[rtaCtr] = accu / ((float)chunkSize);// * 0.7071067811865475; //accomodate for the 3dB from mixdown
    }

    // if ((rtaCtr & 3) == 0) // 25Hz
    // {
        for(int band=0;band<31;++band)
        {
            accu=0;
            for(int bin=0;bin<8;++bin)
                accu+=rtaChannels[band].rta_segments[bin];
            rtaLevels[band] = accu * .0625; // 16 bins
        }
    // }
    rtaCtr++;
    if(rtaCtr==8)
        rtaCtr=0;
}

static void IRAM_ATTR dsp_task(void*)
{
    float accu=0;
    //optional - move task to second core
    vTaskCoreAffinitySet(NULL, 2);

    for(int i=0;i<31;++i)
        dsps_biquad_gen_bpf0db_f32(&rtaCoeff[i], rtaBands[i] / 48000.0, /*2.0**/terz_Q);

//use a GPIO to allow timing measurements
#ifdef TIMING_PROBE
    gpio_set_direction(TIMING_PROBE, GPIO_MODE_OUTPUT);
    gpio_set_level(TIMING_PROBE, 0);
    gpio_set_direction(4, GPIO_MODE_OUTPUT);
#endif

    /** EBU Rec.128 implementation with minimal RAM
     *  we expect one block of audio samples each 10ms
     *  (480 samples at 48kHz) and immediately perform
     *  a number of processing steps on them:
     *   - sample format conversion
     *   - prefilter
     *   - rolloff filter
     *   - sum-of-squares.
     *  (for each channel individually)
     *
     *  Results are stored in an array of 40 elements per channel,
     *  each representing 10ms of SUM(x²) / N.
     *
     *  Each 10 chunks of samples (each 100ms) the 40 segments
     *  are summed together and LUFS is calculated on the last 400ms.
     *  This results in the required 75% (300ms) of overlap without
     *  reprocessing old data.
     *
     *  The result of this operation will be pushed into a queue to be processed in the main task.
     */

    int32_t * bp=NULL;
    int ungatedHistPtr=0;
    float gain = pow(10, 15.0/20.0);

    for(int c=0;c<3;++c)
        for(int i=0;i<SHORT_HIST_LEN;++i)
            lvlShortHistory[c][i] = -99;

    upsamplersF32[0].ptr=0;
    upsamplersF32[1].ptr=0;
    preloadInterpolatorCoeffs();
    initInterpolatorCtx(upsamplers,2);

    int segmentCtr=0;
    int pkCtr=0; int pkHoldCtr=0;
    while(1)
    {
        int gotData = xQueueReceive(inQueue, &bp, 100 / portTICK_PERIOD_MS);
        if (!gotData) {
            printf("DSP task not notified within timeout.\n");
            continue;
        }
        if(!bp)
            continue;
#ifdef TIMING_PROBE
        gpio_set_level(TIMING_PROBE, 1);
#endif
        int32_t * src = bp;
        int32_t dPeak[2];

        copyChannelData(src, processingBuffer, chunkSize, &dPeak[0], &dPeak[1]);
        calcRmsSegment(processingBuffer, gain, 2, segmentCtr);

        // int tpk =  calcTruePeak(src, 0x7fff/4, chunkSize);

        float tpk = calcTruePeakF32(processingBuffer[0], processingBuffer[1]);

        chPkBlkHist[0][pkCtr] = dPeak[0];
        chPkBlkHist[1][pkCtr] = dPeak[1];
        tpkBlockHist[pkCtr++] = tpk;

        for(int i=0;i<10;++i)
        {
            dPeak[0] = CMAX(dPeak[0], chPkBlkHist[0][i]);
            dPeak[1] = CMAX(dPeak[1], chPkBlkHist[1][i]);
            tpk=CMAX(tpk, tpkBlockHist[i]);
        }

        if(pkCtr==10) {
            pkCtr=0;
            chPkHist[0][pkHoldCtr] = dPeak[0];
            chPkHist[1][pkHoldCtr++] = dPeak[1];
            if(pkHoldCtr==10)
                pkHoldCtr = 0;
        }
        int pkh1= dPeak[0];
        int pkh2= dPeak[1];
        for(int i=0;i<10;++i)
        {
            pkh1 = CMAX(pkh1, chPkHist[0][i]);
            pkh2 = CMAX(pkh2, chPkHist[1][i]);
        }
        digiPeakHold[0] = pkh1;
        digiPeakHold[1] = pkh2;
        digiPeak[0] = dPeak[0];
        digiPeak[1] = dPeak[1];



        //channel correlation
        dsps_dotprod_f32(processingBuffer[0], processingBuffer[1], &accu, chunkSize);
        accu/=chunkSize;
        float corr = accu;

        //downmix stereo->mono (results in +3dB)
        dsps_add_f32(processingBuffer[0], processingBuffer[1], processingBuffer[0], chunkSize, 1, 1, 1);

        dsps_dotprod_f32(processingBuffer[0], processingBuffer[0], &accu, chunkSize);
        float totalLevel = accu/chunkSize * 0.5;

        if (fabs(totalLevel)>0.001)
            correlationHist[segmentCtr] = corr / totalLevel * 2; // take mixing gain into account here
        else
            correlationHist[segmentCtr] = 0; //zero on silence

        if ((segmentCtr&3)==0) {
            accu=0;
            for(int i=0;i<40;++i)
                accu+=correlationHist[i];
            accu/=40.0;
            chCorr = accu;
        }

        //precalculations done

        if ((segmentCtr %10 ) == 0)
        {
            float chAccu[2]={0, 0};

            for(int ch=0;ch<2;++ch)
            {
                accu=0;
                for(int i=0;i<40;++i)
                    accu+=r128Ch[ch].k_segments[i];
                chAccu[ch] = accu / 40.0;
            }
            accu=0;
            for(int ch=0;ch<2;++ch)
                accu +=chAccu[ch];

            float lufs_l = -0.691 + 10 * log10(chAccu[0]);
            float lufs_r = -0.691 + 10 * log10(chAccu[1]);
            float lufs_m = -0.691 + 10 * log10(accu);

            lvlShortHistory[0][ungatedHistPtr] = lufs_l;
            lvlShortHistory[1][ungatedHistPtr] = lufs_r;
            lvlShortHistory[2][ungatedHistPtr] = lufs_m;
            tpkHist[ungatedHistPtr] = tpk;

            float lufs_min=0;
            float lufs_max=-999;

            //find dynamic gating threshold
            int nBlocksLimit=0;
            int nBlocksLevel=0;
            accu=0;
            float accu2=0;
            float accu3=0;
            float maxPeak=0;
            for(int i=0;i<SHORT_HIST_LEN;++i)
            {
                maxPeak = CMAX(maxPeak, tpkHist[i]);
                float lkfs = lvlShortHistory[2][i];
                if (lvlShortHistory[2][i] >= threshold_0)
                {
                    accu+= lkfs;
                    accu2+=lvlShortHistory[1][i] - lvlShortHistory[0][i];
                    if (lkfs>=threshold_1)
                    {
                        lufs_max = CMAX(lufs_max, lkfs);
                        lufs_min = CMIN(lufs_min, lkfs);
                        accu3+=lkfs;
                        ++nBlocksLevel;
                    }
                    ++nBlocksLimit;
                }
            }

            if (nBlocksLimit)
                threshold_1 = accu / ((float)nBlocksLimit) - 10;
            else
                threshold_1 = -70;
            if (nBlocksLevel)
            {
                LUFS_I = accu3 / ((float)nBlocksLevel);
            } else {
                LUFS_I = -100;
                LUFS_min=-100;
                lufs_max=-100;
            }

            //NOTE: add correction factors here
            truePeak = 20 * log10(maxPeak);

            int p=ungatedHistPtr;
            accu=0;
            for(int i=0;i<100;++i)
            {
                float val = lvlShortHistory[2][p--];
                if(p<0)
                    p=SHORT_HIST_LEN-1;
                accu+=val;
            }
            accu/=100;
            LUFS_S = accu;
            accu=0;
            p=ungatedHistPtr;
            for(int i=0;i<30;++i)
            {
                float val = lvlShortHistory[0][p--];
                if(p<0)
                    p=SHORT_HIST_LEN-1;
                accu+=val;
            }
            LUFS_avg_l = accu/30.0;


            p=ungatedHistPtr;
            accu=0;
            for(int i=0;i<30;++i)
            {
                float val = lvlShortHistory[1][p--];
                if(p<0)
                    p=SHORT_HIST_LEN-1;
                accu+=val;
            }
            LUFS_avg_r = accu / 30.0;

            LUFS_M = lufs_m;
            LUFS_peak = lufs_max;
            LUFS_min = lufs_min;

            ++ungatedHistPtr;
            if(ungatedHistPtr == SHORT_HIST_LEN)
                ungatedHistPtr = 0;
        }
        rms[0] = r128Ch[0].fastRMS;
        rms[1] = r128Ch[1].fastRMS;

        ++segmentCtr;
        if(segmentCtr>=40) // wrap around all 40 blocks, making the segment array a circular buffer of
            segmentCtr=0;

        //RTA
        calcRTA();

//-- EOF RTA code

#ifdef TIMING_PROBE
        gpio_set_level(TIMING_PROBE, 0);
#endif
        if(xQueuePeek(inQueue, &bp, 0))
            continue;
        taskYIELD();
    }
}



int startDSP(i2s_chan_handle_t channel)
{
    if (dspTask || recTask)
        return 1;
    rx_chan = channel;
    //allocate buffers on HEAP (2x chunksize, i2s buffers are stereo)
    for(int i=0;i<INPUT_BUFFERS;++i)
        ESP_ERROR_CHECK(NULL == (rec_buffer[i] = calloc(2 * chunkSize, sizeof(uint32_t))));

    for(int i=0;i<N_BUFFERS;++i)
    {
        processingBuffer[i] = calloc(chunkSize, sizeof(float));
        ESP_ERROR_CHECK(processingBuffer == NULL);
    }

    inQueue = xQueueCreate(INPUT_BUFFERS, sizeof(int32_t*));

    configASSERT(xTaskCreate(dsp_task     , "EBU DSP", 4096, NULL, 5, &dspTask));
    configASSERT(xTaskCreate(audio_rx_task, "I2S REC", 4096, NULL, 5, &recTask));
    return 0;
}

void reconfigureDSP();
