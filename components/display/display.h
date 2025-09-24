#pragma once
#include <esp_err.h>

/**
 * Depending on the GAL version (if any) installed on the boards different
 * pinouts and clock frequencies are to be used.
 * Leave GAL_VER undefined for a board without GAL (backwards compatible)
 * GAL version 1 is the first PLL enabled board with original GAL equations
 * GAL version 2 is to be used with the new equations, GAL version 2.0 is the original board
 * and 2.1 is the new board. These GALs differbut are compatible in their effective pinmap
 */

//#define DISPLAY_640x400
#define DISPLAY_320x256

#ifdef DISPLAY_640x400

#define DISPLAY_WIDTH  640
#define HSYNC_CYCLES    16

#define DISPLAY_HEIGHT 400
#define VSYNC_LINES      4

#elif defined(DISPLAY_320x256)
#define DISPLAY_WIDTH  320
#define HSYNC_CYCLES    16

#define DISPLAY_HEIGHT 256
#define VSYNC_LINES      4

#define DISPLAY_CLK 3100000

#endif

#define GAL_VER 2

#define GPIO_DISPLAY_PXCK 17

#ifndef GAL_VER

//this board (no GAL) was disassembled but the mapping might be useful, still
#define DISPLAY_PPB    2
#define DISPLAY_CLK 40000000

#define GPIO_DISPLAY_D0_A 15
#define GPIO_DISPLAY_D1_A 16
#define GPIO_DISPLAY_HS   18
#define GPIO_DISPLAY_VS   19

#define GPIO_DISPLAY_D0_B GPIO_NUM_NC
#define GPIO_DISPLAY_D1_B GPIO_NUM_NC

#else

#define DISPLAY_PPB    4

//all versions with GAL use this clock speed
#ifndef DISPLAY_CLK
#define DISPLAY_CLK  8500000
#endif

#if GAL_VER == 1
//GAL equations V1
#define GPIO_DISPLAY_PXCK 17

#define GPIO_DISPLAY_D0_A 21
#define GPIO_DISPLAY_D1_A 22

#define GPIO_DISPLAY_D0_B 15
#define GPIO_DISPLAY_D1_B 16

#define GPIO_DISPLAY_HS   18
#define GPIO_DISPLAY_VS   19
//-------------------

#elif GAL_VER == 2
//GAL equations V2

#define GPIO_DISPLAY_D0_A 15
#define GPIO_DISPLAY_D1_A 16

#define GPIO_DISPLAY_D0_B 21
#define GPIO_DISPLAY_D1_B 22

#define GPIO_DISPLAY_HS   18
#define GPIO_DISPLAY_VS   19

#endif
#endif

#define HS_IDLE 1
#define VS_IDLE 1

esp_err_t display_init();
void init_buffer(void * bp);
void _setPx(int x, int y, int b);
void setPx(int x, int y, int b);
void scrollWolf(int offset, int line);
void _scrollDisplay(int rows);
void copyBuffer(int i);
void copyBufferY(int buffer, int y0);
void orBuffer(int i);
void clearBuffer(int i);
void showLogo();

typedef struct gBuff {
    int w; ///< width  in pixel
    int h; ///< height in pixel
    int s; ///< row stride in u32 words
    uint32_t * d; ///< data pointer (zero if invalid)
} gBuff_t;

void blitBuffer(gBuff_t  *, int x0, int y0);

extern unsigned char drawingBuffer[];
