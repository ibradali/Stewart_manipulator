
#ifndef SH1106_H
#define SH1106_H

#include <stdio.h>
#include "stm32f4xx_hal.h"

#define SH1106_WIDTH (uint8_t) 132
#define SH1106_HEIGHT (uint8_t) 64

#define SH1106_DEV_ADDR (uint8_t) (0x3C<<1)

#define SH1106_SET_PAGE_ADDR_CMD (uint8_t) 0xB0
#define SH1106_SET_LOW_COL_ADDR_CMD (uint8_t) 0x00
#define SH1106_SET_HIGH_COL_ADDR_CMD (uint8_t) 0x10
#define SH1106_TURN_DISP_ON_CMD (uint8_t) 0xAF
#define SH1106_DISP_OFF_CMD (uint8_t) 0xAE
#define SH1106_SET_CLKDIV_CMD (uint8_t) 0xD5
#define SH1106_SET_MULTIPLEX_CMD (uint8_t) 0xA8
#define SH1106_SET_DISP_OFFSET_CMD (uint8_t) 0xD3
#define SH1106_SET_START_LINE_CMD (uint8_t) 0x40
#define SH1106_SET_CONTROL_MODE (uint8_t) 0xAD
#define SH1106_SET_CONTROL (uint8_t) 0x8B
#define SH1106_SET_CONTRAST_CMD_MODE (uint8_t) 0x81
#define SH1106_SET_CONTRAST (uint8_t) 0xCF
#define SH1106_SET_PIN_CONFIG_CMD (uint8_t) 0xDA
#define SH1106_SET_SEG_REMAP_CMD (uint8_t) 0xA1
#define SH1106_SET_PRECHARGE_CMD_MODE (uint8_t) 0xD9
#define SH1106_SET_PRECHARGE_CMD (uint8_t) 0x1F
#define SH1106_SET_VCOM_CMD_MODE (uint8_t) 0xDB
#define SH1106_SET_VCOM_CMD (uint8_t) 0x40
#define SH1106_SET_DISP_OFF_CMD (uint8_t) 0xA4
#define SH1106_SET_NORMAL_DISP_CMD (uint8_t) 0xA6






void send_command(uint8_t command);
void disp_init(void);
void disp_data(void);
void buff_init(void);
void display_string(uint8_t line, char* str);
void draw_point(uint8_t x, uint8_t y);



#endif
