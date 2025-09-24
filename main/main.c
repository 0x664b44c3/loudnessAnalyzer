/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */


#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "../components/display/display.h"
#include "../components/fonts/font.h"
#include <esp_task_wdt.h>
#include "bootscreen.h"
#include <string.h>
#include <ctype.h>
#include <math.h>
#include "driver/gpio.h"

#include "driver/i2s_std.h"

#include "dsp.h"

volatile int cx=0;
volatile int cy=0;
volatile int cursor_blink=0;
#define FONT_W 8
#define FONT_H 16
#define CONSOLE_FONT fntFont8x16
#define CONS_ROWS  (DISPLAY_HEIGHT / FONT_H)
#define CONS_COLS  (DISPLAY_WIDTH  / FONT_W)

#define CMIN(a, b) ((a<b)?a:b)
void _setPx(int x, int y, int b);

enum {
    BUF_ASSEMBLE=0,
    BUF_TEXT,
    BUF_METER,
    BUF_GRAPH,
    BUF_GRAT,
};

gBuff_t gfxBuffer[8];

void buffer_puttext(gBuff_t * buffer, int row, int col, const char * txt, int len, int fontAttr)
{
    struct fontInfo *fi = &fonts[fontAttr & fntTypeFaceMask];
    int nCol = buffer->w / 8;//fi->chWidth;
    if (fi->chWidth>8)
        return; // only 8px wide fonts supported for now (all narrower fonts are spaced apart
    if (len==0)
    {
        len = strlen(txt);
    }
    len = CMIN(len, nCol - col);
    if (len<=0)
        return;
    char  *glyphData = &fi->fontData;

    for(int v=0;v<fi->chHeight;++v)
    {
        char * dptr = &buffer->d[(row*fi->chHeight) + v * buffer->s];
        dptr+=col;
        for(int i=0;i<len;++i) {
            int glyphId = txt[i];
            glyphId -= fi->glyphOffset;
            if ((glyphId<0) || (glyphId >= fi->numGlyphs))
                continue;
            char  gData = 0;//glyphData[glyphId+v];
            if(fontAttr & fntInvert)
                gData^=0xff;
            *dptr++ = gData;
        }
    }
}

int cursor_start=FONT_H-2;
int cursor_height=2;
static void storeCursor()
{

}

static void doCursor(int onOff)
{
    int x=((cx>=CONS_COLS)?CONS_COLS-1:cx)*FONT_W;
    int y=cy*FONT_H;
    //mode 1: nontransparent cursor
    for(int v=cursor_start;v<cursor_start+cursor_height;++v)
    {
        for(int u=0;u<FONT_W;++u)
            _setPx(x+u,y+v,onOff);
    }
}

void moveCursor(int row, int col)
{
    if (cursor_blink)
        doCursor(0);
    if (row>=0)
        cy=row;
    if (col>=0)
        cx = col;
    doCursor(cursor_blink);
}


void console_task(void*p1)
{
    while(1)
    {

        cursor_blink = 1-cursor_blink;
        doCursor(cursor_blink);
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
}

void cons_writeX(const char * txt, int l)
{
    static char esccode=0;
    if (cursor_blink)
        doCursor(0);

    int x=cx*FONT_W; int y=cy*FONT_H;
    for (int i=0;i<l;++i)
    {
        char c = txt[i];
        if (esccode)
        {
            if (isalpha(c))
            {
                esccode=0;
            }
            continue;
        }

        switch(c) {
        case 27:
            esccode = 0xff;
            break;
        case '\n':
            if (cy>=(CONS_ROWS-1))
            {
                _scrollDisplay(FONT_H);
            }
            else
            {
                cy++;
            }
            cx=0;
            break;
        case '\r':
            cx=0;
            break;
        default:
            if (cx>=CONS_COLS)
            {
                if (cy>=(CONS_ROWS-1))
                {
                    _scrollDisplay(FONT_H);
                }
                else
                {
                    cy++;
                }

                cx=0;
                y=cy*FONT_H;
            }
            x=cx*FONT_W;
            drawGlyph(x,y,CONSOLE_FONT,c,1);
            ++cx;
            break;
        }

    }
    if (cursor_blink)
        doCursor(1);
}

void text(int x, int y, int text)
{
    drawText(x * FONT_W, y * FONT_H,CONSOLE_FONT,text,1);
}
void cons_write(const char * txt)
{
    cons_writeX(txt, strlen(txt));
}

void app(void*);

int writeFn(void * tag, const char * d, int l)
{
    cons_writeX(d,l);
    if (tag)
        cons_write("\n");
    return l;
}
void app_main(void*)
{
    printf("Hello world!\n");
    // fflush(stdout);
    // fclose(stdout);
    // fwopen(NULL,&writeFn);

    moveCursor(0,0);
    printf("Display init.\n");
    display_init();
    cons_write("Console ready.\n");
    // xTaskCreate(console_task, "console_task", 4096, NULL, 5, NULL);

    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }

    printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    // printf("Display buffers at [0]: %p", display_buffer);
    // printf("                   [1]: %p", display_buffer[1]);

    // esp_task_wdt_add(NULL); //add current thread to WDT watch
    // esp_task_wdt_reset();

    vTaskDelay(200 / portTICK_PERIOD_MS);
    esp_task_wdt_reset();
    vTaskDelay(1000 / portTICK_PERIOD_MS);    
    // xTaskCreate(app, "audioAnalyzer", 4096, NULL, 5, NULL);
    app(NULL);
    // while(1)
    // {
    //     taskYIELD();
    //     esp_task_wdt_reset();
    // }
}


static i2s_chan_handle_t rx_chan;

extern volatile int nBuff;

static i2s_chan_handle_t    rx_chan;   // I2S rx channel handler

gBuff_t allocGfxBuffer(int w, int h)
{
    gBuff_t ret;
    ret.w = w;
    ret.h = h;
    ret.s = (w + 31) / 32;
    ret.d = (uint32_t*)calloc(ret.s * ret.h, sizeof(uint32_t));
    return ret;
}

void gbOverlay(gBuff_t * dst, gBuff_t * src)
{
    uint32_t *dp = dst->d;
    uint32_t *sp = src->d;
    int nWords = CMIN(dst->s, src->s) * CMIN(dst->h, src->h);
    if(dst->s == src->s)
    { // simple case, just go along both buffers
        for(int i=0;i<nWords;++i)
            *dp++|=*sp++;
    }
    else
    { //kein bock drauf gerade
    }
}

int allocBuffer(int i, const char * n, int w, int h){
    char txt[32];
    sprintf(txt, " %02d: %12.12s..", i, n);
    cons_write(txt);
    gfxBuffer[i] = allocGfxBuffer(w, h);
    if (NULL==gfxBuffer[i].d) {
        cons_write("ERROR.\n");
        return ESP_ERR_NO_MEM;
    }
    else
    {
        sprintf(txt, "Ok @%p\n", gfxBuffer[i].d);
        cons_write(txt);
    }
    return 0;
}

void dspTask(void*);
extern volatile float LUFS_M;

void drawCorrelationBox(int x, int y, int w, const char * lbl) {

    for(int u=0;u<w;++u)
    {
        _setPx(x+u, y   , 1);
        _setPx(x+u, y+11, 1);
        for(int v=1;v<11;++v)
        {
            _setPx(x+u,y+v,(u<17));
        }
    }
    for(int v=0;v<12;++v)
    {
        _setPx(x,y+v,1);
        _setPx(x+w-1,y+v,1);
    }
    int l = strlen(lbl);
    if (l>2)
        l=2;
    int u = 9-l*4;
    for(int i=0;i<l;++i)
        drawGlyph(x+u+i*8,y+2,fntFont8x8|fntInvert, lbl[i], 1);
}

void drawDisplayBox(int x, int y, const char * text)
{
    int w = strlen(text)*6;
    for(int u=0;u<w+2;++u)
    {
        _setPx(x+u,y, 1);
        _setPx(x+u,y+30,1);
    }
    for(int v=0;v<31;++v)
    {
        _setPx(x,y+v,1);
        _setPx(x+w+1,y+v,1);
    }
    drawText(x+1, y+1, fntFont5x7|fntInvert, text, 1);
    drawText(x+1, y+11, fntFont12x16, " --- ",1);
}
void drawDisplayBoxH(int x, int y, int h, const char * text)
{
    int w = strlen(text)*6;
    for(int u=0;u<w+2;++u)
    {
        _setPx(x+u,y, 1);
        _setPx(x+u,y+h-1,1);
    }
    for(int v=0;v<h;++v)
    {
        _setPx(x,y+v,1);
        _setPx(x+w+1,y+v,1);
    }
    drawText(x+1, y+1, fntFont5x7|fntInvert, text, 1);
    drawText(x+1, y+11, fntFont12x16, " --- ",1);
}

#ifndef ATTR_IRAM
#define ATTR_IRAM
#endif

void ATTR_IRAM drawBarX(int x, int y, int h, int m)
{
    for(int v=0;v<h;++v)
        for(int u=0;u<2;++u)
        {
            int on;
            switch(m)
            {
            case 0:
                on=0;
                break;
            case 1:
                on=1;
                break;
            case 2:
                on = (u ^ v) & 1;
                break;
            case 3:
                on = (u ^ v ^ 1) & 1;
                break;
            case 4:
                on = (v==0);
                break;
            case 5:
                on = (v==h-1);
                break;
            }
                _setPx(x+u, y+v, on);
        }
}
void ATTR_IRAM drawBar(int x, int y, int h, int on)
{
    for(int v=0;v<h;++v)
    {
        _setPx(x  , y+v, on);
        _setPx(x+1, y+v, on);
    }
}

void app (void*) {

    gpio_set_direction(4, GPIO_MODE_OUTPUT);


    gpio_set_direction(34, GPIO_MODE_INPUT);


    char *txt= pvPortMalloc(128);
    int fAudio = 0;

    i2s_std_config_t std_cfg;
    std_cfg.clk_cfg.sample_rate_hz = 48000;
    std_cfg.clk_cfg.clk_src = I2S_CLK_SRC_DEFAULT;
    std_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;

    std_cfg.slot_cfg.data_bit_width = 32;     /*!< I2S sample data bit width (valid data bits per sample) */
    std_cfg.slot_cfg.slot_bit_width = 32;     /*!< I2S slot bit width (total bits per slot) */
    std_cfg.slot_cfg.slot_mode      = I2S_SLOT_MODE_STEREO;
    std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_BOTH;          /*!< Select the left, right or both slot */
    std_cfg.slot_cfg.ws_width = 16;           /*!< WS signal width (i.e. the number of BCLK ticks that WS signal is high) */
    std_cfg.slot_cfg.ws_pol=false;             /*!< WS signal polarity, set true to enable high lever first */
    std_cfg.slot_cfg.bit_shift=true;          /*!< Set to enable bit shift in Philips mode */
    std_cfg.slot_cfg.msb_right=0;

    std_cfg.gpio_cfg.mclk = I2S_GPIO_UNUSED;
    std_cfg.gpio_cfg.bclk = GPIO_NUM_14;
    std_cfg.gpio_cfg.ws   = GPIO_NUM_27;
    std_cfg.gpio_cfg.dout = GPIO_NUM_NC;
    std_cfg.gpio_cfg.din  = GPIO_NUM_26;
    std_cfg.gpio_cfg.invert_flags.mclk_inv = false;
    std_cfg.gpio_cfg.invert_flags.bclk_inv = false;
    std_cfg.gpio_cfg.invert_flags.ws_inv   = false;


    fflush(stdout);

    printf("INIT APP\n");

    clearBuffer(0);
    copyBuffer(0);
    moveCursor(0,0);
    cons_write("Alloc Gfx buffers:\n");
    ESP_ERROR_CHECK(allocBuffer(BUF_ASSEMBLE, "BGND", DISPLAY_WIDTH, DISPLAY_HEIGHT));
    ESP_ERROR_CHECK(allocBuffer(BUF_TEXT, "Text", DISPLAY_WIDTH, DISPLAY_HEIGHT));
    ESP_ERROR_CHECK(allocBuffer(BUF_METER, "Meter", DISPLAY_WIDTH, 96));
    ESP_ERROR_CHECK(allocBuffer(BUF_GRAPH, "Scope", DISPLAY_WIDTH, 128));
    ESP_ERROR_CHECK(allocBuffer(BUF_GRAT, "Graticule", DISPLAY_WIDTH, 128));

    //ESP_ERROR_CHECK(allocBuffer(BUF_GRAT,    "Drawing", DISPLAY_WIDTH, DISPLAY_HEIGHT));
    fflush(stdout);
    cons_write("Config I2S peripheral 0: ");
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_SLAVE);
    esp_err_t ret = i2s_new_channel(&chan_cfg, NULL, &rx_chan);

    if (ret != ESP_OK)
    {
        cons_write("FAIL\n");
        return;
    }
    else
        cons_write("OK\n");

    cons_write("Setup Rx channel..");
    ret = i2s_channel_init_std_mode(rx_chan, &std_cfg);
    if (ret == ESP_OK) {
        cons_write("OK\n");
    }
    else
    {
        cons_write("FAILED\n");
        return;
    }

    cons_write("Starting DSP tasks...\n");
    ret = startDSP(rx_chan);

    cons_write("Starting audio rx..");
    ret = i2s_channel_enable(rx_chan);
    if (ret == ESP_OK) {
        cons_write("OK\n");
    }
    else
    {
        cons_write("FAILED\n");
    }

    cons_write("Minimum free heap size: ");
    sprintf(txt, "%" PRIu32 " Bytes\n", esp_get_minimum_free_heap_size());
    cons_write(txt);

    int bctr=0, octr=0;
    int blnk=0;
    /* DSP processing buffers of 10ms each*/
    /*
     * int samples ==convert==> buffer0, buffer 1 (float)
     * buffer0,1 ==prefiler==> buffer2,3
     * buffer2,3 ==lowcut==> buffer0,1
     * buffer0 ==SEUM(x2)==> accu_ch0, accu_ch1
     */
    //allocate 10ms of float samples (one channel)
    float *dspBuffer[4];
    float chAccu[2];

    sprintf(txt, "%" PRIu32 " Bytes free\n", esp_get_minimum_free_heap_size());
    cons_write(txt);
    //alloc lufs realted buffers

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    clearBuffer(0);
    copyBuffer(0);
    for(int x=0;x<320;++x)
        for(int y=0;y<256;++y)
            _setPx(x,y,1);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    clearBuffer(0);
    copyBuffer(0);
    int bw = 64;
    int by=48;

    drawDisplayBox(bw*4,15,"True Peak ");

    drawDisplayBox(bw*0,by,"PGM LKFS M");
    drawDisplayBox(bw*1,by,"PGM Pk LU ");
    drawDisplayBox(bw*2,by,"LRA (1min)");
    drawDisplayBox(bw*3,by,"LKFS(I)   ");
    drawDisplayBox(bw*4,by,"10s avg   ");

    for(int x=0;x<320;++x)
        for(int v=0;v<10;++v)
            _setPx(x,232+v,1);

    for(int i=0;i<5;++i)
        for(int v=0;v<24;++v)
            _setPx(i*64,232+v,1);

    for(int i=0;i<5;++i)
    {
        for(int v=11;v<24;v+=2)
            _setPx(i*64+63,232+v,1);

        _setPx(i*64,232,0);
        _setPx(i*64+63,232,0);
    }

    for(int i=0;i<5;++i)
        for(int v=2;v<10;v+=2)
            _setPx(i*64+63,232+v,0);

    drawText(64*0+3, 233, fntFont5x7|fntInvert, "Graph", 1);
    drawText(64*1+3, 233, fntFont5x7|fntInvert, "Inp Gain", 1);
    drawText(64*2+3, 233, fntFont5x7|fntInvert, "Peak hold", 1);
    drawText(64*3+3, 233, fntFont5x7|fntInvert, "---", 1);
    drawText(64*4+3, 233, fntFont5x7|fntInvert, "System", 1);

    drawText(64*0+4, 234+12, fntFont8x8, "RTA", 1);
    drawText(64*1+4, 234+12, fntFont8x8, "+18dB", 1);
    drawText(64*2+4, 234+12, fntFont8x8, "10s", 1);

    drawText(64*4+4+8, 234+12, fntFont8x8, "00:00", 1);


    drawCorrelationBox(0,34,126,"C");
    drawCorrelationBox(128,34,126,"SD");



    uint32_t data[]={0x8badf00d,0xc0febabe};
    uint16_t *partial = (uint16_t*)data;
    for(int i=0;i<4;++i)
        printf("Block %d: 0x%04x\n", i, *partial++);

    while(1) {
        octr=bctr;
        bctr = nBuff;
        if (bctr>octr) {
            fAudio = ((bctr-octr) * 480) * 1000 / 100;
        }

        moveCursor(0,0);

        cons_write("L");
        moveCursor(1,0);
        cons_write("R");

        float rms_l = 10*log10(rms[0]);
        float rms_r = 10*log10(rms[1]);
        // float rms_m = 10*log10(rms[0]+rms[1]);

        //adjust for gain on the fly
        float pk_l = 20*log10(digiPeak[0] / 2147483392.0) + 15;
        float pk_r = 20*log10(digiPeak[1] / 2147483392.0) + 15;
        float pkh_l = 20*log10(digiPeakHold[0] / 2147483392.0) + 15;
        float pkh_r = 20*log10(digiPeakHold[1] / 2147483392.0) + 15;
        int peakHoldBar0 = 0;
        int peakHoldBar1 = 0;
        if(pkh_l>-60)
            peakHoldBar0 = (pkh_l+87.0) / 1.5;
        if(pkh_r>-60)
            peakHoldBar1 = (pkh_r+87.0) / 1.5;
        if(peakHoldBar0>57)
            peakHoldBar0=57;
        if(peakHoldBar1>57)
            peakHoldBar1=57;
        for(int i=0;i<58;++i)
        {
            float thresh = ((float)i) * 1.5 - 87;
            drawBarX(12 + i*4,  0, 12, ((peakHoldBar0==i)||(rms_l >= thresh))?1:(pk_l>=thresh)?2:5);
            drawBarX(12 + i*4, 19, 12, ((peakHoldBar1==i)||(rms_r >= thresh))?1:(pk_r>=thresh)?2:4);
        }
        moveCursor(2,0);
        float stDev= LUFS_avg_r - LUFS_avg_l;

        //do this in both directions
        drawBarX(   0 + 68, 40, 4, 1);
        drawBarX(   0 + 72, 40, 4, 1);
        drawBarX( 128 + 68, 40, 4, 1);
        drawBarX( 128 + 72, 40, 4, 1);
        for(int bar=1;bar<13;++bar)
        {
            float thresh_C = bar * (1.0/12.0);
            float thresh_D = bar * 0.5;

            drawBar(   0 + 68-4.0*bar, 36, 8,(bar==0)||(chCorr<=-thresh_C));
            drawBar(   0 + 72+4.0*bar, 36, 8,(bar==0)||(chCorr>= thresh_C));
            drawBar( 128 + 68-4.0*bar, 36, 8,(bar==0)||(stDev <=-thresh_D));
            drawBar( 128 + 72+4.0*bar, 36, 8,(bar==0)||(stDev >= thresh_D));
        }

        float tpk = truePeak;
        if(tpk>-99)
            sprintf(txt, "%+05.1f", tpk);
        else
            strcpy(txt, "  -.-");
        drawText(bw*4+1,15+11, fntFont12x16, txt,1);


        sprintf(txt, "%+05.1f", LUFS_M);
        if (LUFS_M>-99)
            drawText(bw*0+1,by+11, fntFont12x16, txt,1);
        else
            drawText(bw*0+1,by+11, fntFont12x16, "---.-",1);


        sprintf(txt, "%+05.1f", LUFS_peak);
        if (LUFS_peak>-99)
            drawText(bw*1+1,by+11, fntFont12x16, txt,1);
        else
            drawText(bw*1+1,by+11, fntFont12x16, "---.-",1);

        float range = LUFS_peak-LUFS_min;
        if ((range>-99) && (range<99))
            sprintf(txt, "%5.1f", range);
        else
            strcpy(txt,"  0.0");
        drawText(bw*2+1,by+11, fntFont12x16, txt,1);

        sprintf(txt, "%+05.1f", LUFS_I);
        if (LUFS_I>-99)
            drawText(bw*3+1,by+11, fntFont12x16, txt,1);
        else
            drawText(bw*3+1,by+11, fntFont12x16, "---.-",1);


        if (LUFS_S>-99)
            sprintf(txt, "%+05.1f", LUFS_S);
        else
            strcpy(txt,"---.-");
        drawText(bw*4+1,by+11, fntFont12x16, txt,1);

        for(int band=0;band<31;++band)
        {
            float db = 10 * log10(rtaLevels[band]);
            for(int lvl=0;lvl<30;++lvl)
            {
                float threshold = ((float)lvl)*2;
                int onOff = (db+threshold>=0);
                int x = band*10;
                int y = 100 + lvl * 4;
                for(int u=0;u<6;++u)
                    for(int v=0;v<2;++v)
                        _setPx(x+u,v+y,onOff|((v==1)&&(u&1)));
            }
        }
        ++blnk;
        // esp_task_wdt_reset();
        vTaskDelay(10 / portTICK_PERIOD_MS);
        taskYIELD();
    }

    esp_restart();
}
