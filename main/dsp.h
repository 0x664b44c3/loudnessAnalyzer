#ifndef DSP_H
#define DSP_H
#include <inttypes.h>

#define MAX_CHANNELS 2

typedef struct r128Channel {
    float filter1_z[2];
    float filter2_z[2];
    float * buffer[2];
    float k_segments[40];
    float rms_segments[40];
    float fastRMS; ///< square root not calculated at this point
    int current_segment;
    int active_buffer;
} r128Ch_t;


typedef struct rtaContext {
    float filter_w[2];
    float rta_segments[16];
} rtaChannel_t;


volatile extern float rms[2];
volatile extern int samplePeak[2];
volatile extern int peakHold[2];
volatile extern float chCorr;
extern float rtaLevels[];

typedef struct dspResults
{
    int32_t digitalPeak;
    float weightedSum[MAX_CHANNELS];
    float unweightedSum[MAX_CHANNELS];
} dspResults_t;

volatile extern int digiPeak[];
volatile extern int digiPeakHold[];
volatile extern float LUFS_S;
volatile extern float LUFS_avg_l;
volatile extern float LUFS_avg_r;
volatile extern float LUFS_I;
volatile extern float LUFS_peak;
volatile extern float LUFS_min;
volatile extern float truePeak;


int startDSP(i2s_chan_handle_t channel);
void configureDSP(float gainDB, int enableRTA, int enableTPk);

#endif // DSP_H
