/*
 * EL display driver for 1BPP 2 pixel per clock EL displays
 */
#include "../bitmaps/ef_wolf.h"
#include <string.h>
#include <stdlib.h>

#include "display.h"
#include <stdint.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <soc/gpio_pins.h>
#include "driver/gpio.h"

#include <soc/lldesc.h>

#include "i2s_parallel.h"

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

#define DBUFFER_STRIDE ((DISPLAY_WIDTH + 7) / 8)
#define DBUFFER_SIZE DBUFFER_STRIDE*DISPLAY_HEIGHT
//pad dbuffer to be an integer multiple of 32bit

unsigned char drawingBuffer[DBUFFER_SIZE];

void _setPx(int x, int y, int b);
#define BUFFER_SIZE (DISPLAY_WIDTH + HSYNC_CYCLES) * (DISPLAY_HEIGHT + VSYNC_LINES) / DISPLAY_PPB

const int buffer_stride = (DISPLAY_WIDTH + HSYNC_CYCLES) / DISPLAY_PPB;
const int buffer_rows   = DISPLAY_HEIGHT + VSYNC_LINES;

static uint8_t display_outBuf[BUFFER_SIZE] = {0};

static uint8_t display_outBufB[BUFFER_SIZE] = {0};


// void IRAM_ATTR copyBuffer(int buffer) {
//     (void) buffer;
//     const int loadsPerByte = 8/DISPLAY_PPB;
//     unsigned char srcMask = (0xff >> (8- DISPLAY_PPB));
//     int dx=0;
//     int dy=0;
//     int offset = 0;
//     for (int i=0;i<DBUFFER_SIZE;++i)
//     {
//         unsigned char d = drawingBuffer[i];
//         for(int j=0;j<loadsPerByte;++j)
//         {
//             display_outBuf[(offset++)^2] = 0xc0 | (d & srcMask);
//             d>>=DISPLAY_PPB;
//             dx+=DISPLAY_PPB;
//             if (dx>=DISPLAY_WIDTH)
//             {
//                 dx=0;
//                 offset= ++dy * buffer_stride;
//             }
//         }
//     }
// }

#ifndef CMIN
#define CMIN(a, b) ((a<b)?a:b)
#endif
void blitBuffer(gBuff_t  * buf, int x0, int y0)
{
    const int loadsPerWord = 32/DISPLAY_PPB;
    uint32_t srcMask = (0xffffffff >> (32 - DISPLAY_PPB));
    int dx=0;
    //linewise copy
    for(int v=0;v<buf->h;++v)
    {
        dx = x0;
        uint32_t * src = &buf->d[buf->s * v]; //first word of source buffer
        int offset = buffer_stride * v;
        for(int i=0;(i<buf->s) && (dx<DISPLAY_WIDTH);++i)
        {
            uint32_t d = *src++;
            for(int j=0;j<loadsPerWord;++j)
            {
                display_outBuf[(offset++)^2] = 0xc0 | (d & srcMask);
                d>>=DISPLAY_PPB;
                dx+=DISPLAY_PPB;
                if(dx>=DISPLAY_WIDTH) // reached end of active line
                    break;
            }
        }
    }
}

void configure_io (int *pins, int count) {
    for (int i=0;i<count;++i)
        if (pins[i]!=-1)
            gpio_reset_pin(pins[i]);
    for (int i=0;i<count;++i)
        if (pins[i]!=-1)
            gpio_set_direction(pins[i], GPIO_MODE_OUTPUT);
}

esp_err_t display_init()
{
    init_buffer(display_outBuf);


    i2s_parallel_buffer_desc_t bufdesc;
    bufdesc.memory = display_outBuf;
    bufdesc.size = BUFFER_SIZE;

    //create BUS
    i2s_parallel_config_t cfg = {
        .gpio_bus = {
            GPIO_DISPLAY_D0_A, // bit 0
            GPIO_DISPLAY_D1_A, // bit 1
            GPIO_DISPLAY_D0_B,
            GPIO_DISPLAY_D1_B,
            GPIO_NUM_NC,
            GPIO_NUM_NC,
            GPIO_DISPLAY_HS,
            GPIO_DISPLAY_VS,   // bit 7
        },
        .gpio_clk = GPIO_DISPLAY_PXCK,
        .clkspeed_hz = DISPLAY_CLK, // results in a divider of 255 (the maximum)
        .clk_inv = 1, // TODO: Kconfig!
        .clk_x2  = 1,
        .bits = I2S_PARALLEL_BITS_8,
        .buf = &bufdesc
    };

    configure_io(cfg.gpio_bus, cfg.bits);

    i2s_parallel_setup(&I2S1, &cfg);
    return ESP_OK;
}

void showLogo() {
    init_buffer(display_outBuf);
    for(int y=0; y<350;++y)
        for(int x=0; x<272;++x)
            _setPx(x,y, imgdata_ef_wolf[x/8+y*34]&(0x80>>(x&7)));
}

void init_buffer(void * bp)
{
    uint8_t * buf = (uint8_t*)bp;
    int active_bytes = DISPLAY_WIDTH / DISPLAY_PPB;
    unsigned int offset=0;
    for(int v=0;v<buffer_rows;++v)
    {
        for(int u=0;u<buffer_stride;++u)
        {
            uint8_t vs = (v>=DISPLAY_HEIGHT)?0x00:0x80;
            // if ((v == DISPLAY_HEIGHT) && u< 16)
            //     vs=0;
            // if ((v == 0) && u< 16)
            //     vs=1;
            uint8_t vid = 0;//u&7;
            uint8_t d = vid | vs | ((u<active_bytes)?0x40:0x00);
            buf[offset^2] = d;
            ++offset;
        }
    }
}

void IRAM_ATTR _scrollDisplay(int rows) {
    if (rows<1)
        return;
    uint8_t* d1 = &display_outBuf[buffer_stride * rows];
    int sz = (DISPLAY_HEIGHT - rows) * buffer_stride;
    memcpy(display_outBuf, d1, sz);
    for(int i=0;i<rows;++i)
        for(int u=0;u<DISPLAY_WIDTH;++u)
            _setPx(u, DISPLAY_HEIGHT-1-i,0);
}


void IRAM_ATTR _scrollDisplay2(int rows, int top) {
    if ((top<0)||((top+rows)>DISPLAY_HEIGHT))
        return;
    if (rows<1)
        return;
    uint8_t* d1 = &display_outBuf[buffer_stride * rows];
    int sz = (DISPLAY_HEIGHT - rows) * buffer_stride;
    memcpy(&display_outBuf[top], d1, sz);
    for(int i=1;i<=rows;++i)
        for(int u=0;u<DISPLAY_WIDTH;++u)
            _setPx(0, top+rows-i,0);
}

void IRAM_ATTR copyBuffer(int buffer) {
    (void) buffer;
    const int loadsPerByte = 8/DISPLAY_PPB;
    unsigned char srcMask = (0xff >> (8- DISPLAY_PPB));
    int dx=0;
    int dy=0;
    int offset = 0;
    for (int i=0;i<DBUFFER_SIZE;++i)
    {
        unsigned char d = drawingBuffer[i];
        for(int j=0;j<loadsPerByte;++j)
        {
            display_outBuf[(offset++)^2] = 0xc0 | (d & srcMask);
            d>>=DISPLAY_PPB;
            dx+=DISPLAY_PPB;
            if (dx>=DISPLAY_WIDTH)
            {
                dx=0;
                offset= ++dy * buffer_stride;
            }
        }
    }
}


void IRAM_ATTR copyBufferY(int buffer, int y0) {
    (void) buffer;
    const int loadsPerByte = 8/DISPLAY_PPB;
    unsigned char srcMask = (0xff >> (8- DISPLAY_PPB));
    int dx=0;
    int dy=y0;
    int offset = y0 * buffer_stride;

    for (int i=y0*DISPLAY_WIDTH;i<DBUFFER_SIZE;++i)
    {
        unsigned char d = drawingBuffer[i];
        for(int j=0;j<loadsPerByte;++j)
        {
            display_outBuf[(offset++)^2] = 0xc0 | (d & srcMask);
            d>>=DISPLAY_PPB;
            dx+=DISPLAY_PPB;
            if (dx>=DISPLAY_WIDTH)
            {
                dx=0;
                offset= ++dy * buffer_stride;
            }
        }
    }
}

void IRAM_ATTR orBuffer(int buffer) {
    (void) buffer;
    const int loadsPerByte = 8/DISPLAY_PPB;
    unsigned char srcMask = (0xff >> (8- DISPLAY_PPB));
    int dx=0;
    int dy=0;
    int offset = 0;
    for (int i=0;i<DBUFFER_SIZE;++i)
    {
        unsigned char d = drawingBuffer[i];
        for(int j=0;j<loadsPerByte;++j)
        {
            display_outBuf[(offset++)^2] |= 0xc0 | (d & srcMask);
            d>>=DISPLAY_PPB;
            dx+=DISPLAY_PPB;
            if (dx>=DISPLAY_WIDTH)
            {
                dx=0;
                offset = ++dy * buffer_stride;
            }
        }
    }
}

/**
 * @brief copyBufferX - copies a part of the drawing buffer to the screen buffer
 * @note this will grow the copied section to copy full bytes
 * @param buffer - buffer number
 * @param x0 - start x
 * @param y0 - start y
 * @param w - width
 * @param h - height
 */
void IRAM_ATTR copyBufferX(int buffer, int x0, int y0, int w, int h) {
    const int loadsPerByte = 8/DISPLAY_PPB;
    unsigned char srcMask = (0xff >> (8- DISPLAY_PPB));

    (void) buffer;
    if (x0<0)
        x0=0;
    if (y0<0)
        y0=0;
    if (x0>=DISPLAY_WIDTH)
        return;
    if (y0>=DISPLAY_HEIGHT)
        return;
    int ex0 = x0 & (~7);

    // for(int v=0;v<h;++v)
    // {
    //     char * src = &drawingBuffer[DBUFFER_STRIDE * (y0+v) + dx];
    //     int dstOffset = DBUFFER_SIZE * (y0 + v) + (x0&(~7)) / DISPLAY_PPB;
    //     for (int i=0;i<DBUFFER_SIZE;++i)
    // {
    //     unsigned char d = drawingBuffer[i];
    //     for(int j=0;j<loadsPerByte;++j)
    //     {
    //         display_outBuf[(offset++)^2] = 0xc0 | (d & srcMask);
    //         d>>=DISPLAY_PPB;
    //         dx+=DISPLAY_PPB;
    //         if (dx>=DISPLAY_WIDTH)
    //         {
    //             dx=0;
    //             offset= ++dy * buffer_stride;
    //         }
    //     }
    // }
}

void IRAM_ATTR _setPx(int x, int y, int b)
{
    if ((x<0) || (x>=DISPLAY_WIDTH) || (y<0) || (y>=DISPLAY_HEIGHT))
        return;
    int offset = (x / DISPLAY_PPB + y * buffer_stride)^2;
    unsigned char bit = 1<<(x&(DISPLAY_PPB-1));
    unsigned char mask = 0xff^ bit;
    if (b==0)
        bit=0;
    display_outBuf[offset] = (display_outBuf[offset] & mask) | bit;
}


void IRAM_ATTR setPx(int x, int y, int b)
{
    const int db_stride=(DISPLAY_WIDTH + 7) / 8;
    if ((x<0) || (x>=DISPLAY_WIDTH) || (y<0) || (y>=DISPLAY_HEIGHT))
        return;
    int offset = (x / 8 + y * db_stride);
    unsigned char bit = 1<<(x&7);
    unsigned char mask = 0xff^ bit;
    if (b==0)
        bit=0;
    drawingBuffer[offset] = (drawingBuffer[offset] & mask) | bit;
}


void scrollWolf(int offset, int line) {
    for(int y=line&1; y<350;y+=2)
        for(int x=0; x<272;++x)
        {
            _setPx(x+offset,y+25, imgdata_ef_wolf[x/8+y*34]&(0x80>>(x&7)));
        }

}
void bitBlt(void * src, int src_stride, int dx, int dy)
{

}

void clearBuffer(int i) {
    memset(drawingBuffer,0,sizeof(drawingBuffer));
}

void orBuffers(void* dst, void* src)
{

}
