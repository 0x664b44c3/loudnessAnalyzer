#include <stdio.h>
#include <string.h>
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_heap_caps.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/i2s_struct.h"
#include "soc/i2s_reg.h"
#include "esp_intr_alloc.h"
#include <soc/lldesc.h>

#define LCD_DATA_WIDTH        8
#define LCD_PIXEL_CLOCK_HZ    20000000
#define DMA_DESCRIPTOR_BUFFER_SIZE  2048
#define DMA_DESC_COUNT        8

typedef struct {
    uint8_t* buf1;
    uint8_t* buf2;
    size_t size;
    volatile uint8_t active_buf;
} double_buffer_t;

double_buffer_t dbuf = {
    .buf1 = NULL,
    .buf2 = NULL,
    .size = DMA_DESCRIPTOR_BUFFER_SIZE,
    .active_buf = 0
};

lldesc_t dma_desc[DMA_DESC_COUNT];

void IRAM_ATTR i2s_isr(void* arg);

void init_lcd() {
    dbuf.buf1 = (uint8_t*)heap_caps_malloc(dbuf.size, MALLOC_CAP_DMA);
    dbuf.buf2 = (uint8_t*)heap_caps_malloc(dbuf.size, MALLOC_CAP_DMA);

    esp_lcd_i80_bus_handle_t i80_bus;
    esp_lcd_panel_io_handle_t io_handle;

    esp_lcd_i80_bus_config_t bus_config = {
                                           .dc_gpio_num = 21,
                                           .wr_gpio_num = 22,
                                           .data_gpio_nums = {23, 24, 25, 26, 27, 28, 29, 30},
                                           .bus_width = LCD_DATA_WIDTH,
                                           .max_transfer_bytes = DMA_DESCRIPTOR_BUFFER_SIZE,
                                           .psram_trans_align = 64,
                                           .sram_trans_align = 4,
                                           .clk_src = LCD_CLK_SRC_PLL160M,
                                           };

    esp_lcd_new_i80_bus(&bus_config, &i80_bus);

    esp_lcd_panel_io_i80_config_t io_config = {
        .cs_gpio_num = 19,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .trans_queue_depth = 10,
        .dc_levels = {
            .dc_idle_level = 0,
            .dc_cmd_level = 0,
            .dc_dummy_level = 0,
            .dc_data_level = 1,
        },
        .on_color_trans_done = NULL,
        .user_ctx = NULL,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };

    esp_lcd_new_panel_io_i80(i80_bus, &io_config, io_handle);

    for (int i = 0; i < DMA_DESC_COUNT; ++i) {
        dma_desc[i].length = dbuf.size / DMA_DESC_COUNT;
        dma_desc[i].size = dbuf.size / DMA_DESC_COUNT;
        dma_desc[i].buf = (i % 2 == 0) ? dbuf.buf1 : dbuf.buf2;
        dma_desc[i].eof = 1;
        dma_desc[i].owner = 1;
        dma_desc[i].qe.stqe_next = &dma_desc[(i + 1) % DMA_DESC_COUNT];
    }

    WRITE_PERI_REG(I2S0.out_link.addr, (uint32_t)&dma_desc[0]);
    SET_PERI_REG_BITS(I2S0.int_ena.val, 1, 1, I2S_OUT_EOF_INT_RAW);

    esp_intr_alloc(ETS_I2S0_INTR_SOURCE, 0, i2s_isr, NULL, NULL);
}

void IRAM_ATTR i2s_isr(void* arg) {
    if (I2S0.int_st.out_eof) {
        I2S0.int_clr.out_eof = 1;

        uint8_t* buffer = (dbuf.active_buf) ? dbuf.buf1 : dbuf.buf2;
        dbuf.active_buf = !dbuf.active_buf;

        memset(buffer, 0, dbuf.size);

        uint32_t next_buf_addr = (uint32_t)buffer;
        WRITE_PERI_REG(I2S0.out_link.addr, next_buf_addr | I2S_OUTLINK_START);
    }
}

void app_main(void) {
    init_lcd();

    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}













#include <stdio.h>
#include <string.h>
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_heap_caps.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "soc/i2s_struct.h"
#include "esp_intr_alloc.h"
#include "soc/lldesc.h"

#define LCD_DATA_WIDTH        4
#define LCD_PIXEL_CLOCK_HZ    12500000
#define DMA_DESCRIPTOR_BUFFER_SIZE  2048
#define DMA_DESC_COUNT        8
#define FRAMEBUFFER_SIZE      (320 * 240 * 2) // Example framebuffer size for a 320x240 LCD with 16-bit color

typedef struct {
    uint8_t* buf1;
    uint8_t* buf2;
    size_t size;
    volatile uint8_t active_buf;
} double_buffer_t;

double_buffer_t dbuf = {
    .buf1 = NULL,
    .buf2 = NULL,
    .size = DMA_DESCRIPTOR_BUFFER_SIZE,
    .active_buf = 0
};

lldesc_t dma_desc[DMA_DESC_COUNT];

uint8_t* framebuffer1; // First framebuffer
uint8_t* framebuffer2; // Second framebuffer

volatile bool _frame_ready = false;
SemaphoreHandle_t _frame_isr_mutex;

void IRAM_ATTR frame_done_isr(void* arg);

void init_lcd() {
    // Allocate entire framebuffer as DMA-capable memory
    framebuffer1 = (uint8_t*)heap_caps_malloc(FRAMEBUFFER_SIZE, MALLOC_CAP_DMA);
    framebuffer2 = (uint8_t*)heap_caps_malloc(FRAMEBUFFER_SIZE, MALLOC_CAP_DMA);

    _frame_isr_mutex = xSemaphoreCreateMutex();

    esp_lcd_i80_bus_handle_t i80_bus;
    esp_lcd_panel_io_handle_t io_handle;

    esp_lcd_i80_bus_config_t bus_config = {
                                           .dc_gpio_num = 21,
                                           .wr_gpio_num = 22,
                                           .data_gpio_nums = {23, 24, 25, 26, 27, 28, 29, 30},
                                           .bus_width = LCD_DATA_WIDTH,
                                           .max_transfer_bytes = DMA_DESCRIPTOR_BUFFER_SIZE,
                                           .psram_trans_align = 64,
                                           .sram_trans_align = 4,
                                           .clk_src = LCD_CLK_SRC_PLL160M,
                                           };

    esp_lcd_new_i80_bus(&bus_config, &i80_bus);

    esp_lcd_panel_io_i80_config_t io_config = {
        .cs_gpio_num = 19,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .trans_queue_depth = 10,
        .dc_levels = {
            .dc_idle_level = 0,
            .dc_cmd_level = 0,
            .dc_dummy_level = 0,
            .dc_data_level = 1,
        },
        .on_color_trans_done = NULL,
        .user_ctx = NULL,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };

    esp_lcd_new_panel_io_i80(i80_bus, &io_handle);

    // Initialize DMA descriptors to handle segments of the framebuffer
    uint8_t* framebuffer = framebuffer1;
    for (int i = 0; i < DMA_DESC_COUNT; ++i) {
        dma_desc[i].length = DMA_DESCRIPTOR_BUFFER_SIZE;
        dma_desc[i].size = DMA_DESCRIPTOR_BUFFER_SIZE;
        dma_desc[i].buf = framebuffer + i * DMA_DESCRIPTOR_BUFFER_SIZE;
        dma_desc[i].eof = 1;
        dma_desc[i].owner = 1;
        dma_desc[i].qe.stqe_next = &dma_desc[(i + 1) % DMA_DESC_COUNT];
    }

    WRITE_PERI_REG(I2S0.out_link.addr, (uint32_t)&dma_desc[0]);
    SET_PERI_REG_BITS(I2S0.int_ena.val, 1, 1, I2S_OUT_EOF_INT_RAW);

    esp_intr_alloc(ETS_I2S0_INTR_SOURCE, 0, frame_done_isr, NULL, NULL);
}

void IRAM_ATTR frame_done_isr(void* arg) {
    if (I2S0.int_st.out_eof) {
        I2S0.int_clr.out_eof = 1;

        static int segment = 0;

        xSemaphoreTake(_frame_isr_mutex, portMAX_DELAY);
        if (_frame_ready) {
            uint8_t* framebuffer = (dbuf.active_buf) ? framebuffer1 : framebuffer2;
            dbuf.active_buf = !dbuf.active_buf;

            // Fill the buffer with data from the next framebuffer
            dma_desc[segment].buf = framebuffer + segment * DMA_DESCRIPTOR_BUFFER_SIZE;

            _frame_ready = false; // Clear the frame ready flag
        } else {
            // Reuse the current buffer if a new frame is not ready
            dma_desc[segment].buf = (dbuf.active_buf) ? dbuf.buf1 : dbuf.buf2;
        }
        xSemaphoreGive(_frame_isr_mutex);

        // Update DMA descriptor to point to the next buffer segment
        uint32_t next_buf_addr = (uint32_t)dma_desc[segment].buf;
        WRITE_PERI_REG(I2S0.out_link.addr, next_buf_addr | I2S_OUTLINK_START);

        segment = (segment + 1) % DMA_DESC_COUNT;
    }
}

void app_main(void) {
    // Initialize framebuffers with example data
    memset(framebuffer1, 0xFF, FRAMEBUFFER_SIZE); // Fill framebuffer1 with white color
    memset(framebuffer2, 0x00, FRAMEBUFFER_SIZE); // Fill framebuffer2 with black color

    init_lcd();

    while (1) {
        // Main application loop
        // Example: Set _frame_ready flag periodically to swap framebuffers
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        xSemaphoreTake(_frame_isr_mutex, portMAX_DELAY);
        _frame_ready = true;
        xSemaphoreGive(_frame_isr_mutex);
    }
}
