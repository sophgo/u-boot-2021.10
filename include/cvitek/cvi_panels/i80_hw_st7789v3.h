#ifndef _MCU_PARAM_ST7789V3_H_
#define _MCU_PARAM_ST7789V3_H_

#include <cvi_hw_i80.h>

#define PANEL_COMM 0
#define PANEL_DATA	1

HW_I80_CFG_S st7789v3Cfg = {
	.pins = {
		.pin_num = 11,
		.d_pins = {
			{VO_VIVO_D6, VO_MUX_MCU_DATA0},
			{VO_VIVO_D5, VO_MUX_MCU_DATA1},
			{VO_VIVO_D4, VO_MUX_MCU_DATA2},
			{VO_VIVO_D3, VO_MUX_MCU_DATA3},
			{VO_VIVO_D2, VO_MUX_MCU_DATA4},
			{VO_VIVO_D1, VO_MUX_MCU_DATA5},
			{VO_VIVO_D0, VO_MUX_MCU_DATA6},
			{VO_VIVO_D7, VO_MUX_MCU_DATA7},
			{VO_MIPI_TXM1, VO_MUX_MCU_RD},
			{VO_MIPI_RXP5, VO_MUX_MCU_WR},
			{VO_MIPI_TXP1, VO_MUX_MCU_RS},
		}
	},
	.mode = VO_MCU_MODE_RGB565,
	.instrs = {
		.instr_num = 74,
		.instr_cmd = {
			{.delay = 120,  .data_type = PANEL_COMM, .data = 0x11},
			{.delay = 0,   .data_type = PANEL_COMM,	 .data = 0x36},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0x00},
			{.delay = 0,   .data_type = PANEL_COMM,	 .data = 0x3A},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0x05}, //0x101 = 16 bit/pixel, 0x110 = 18bit/pixel
			{.delay = 0,   .data_type = PANEL_COMM,	 .data = 0xB1},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0x00},
			{.delay = 0,   .data_type = PANEL_COMM,	 .data = 0xB2},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0x0C},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0x0C},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0x00},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0x33},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0x33},
			{.delay = 0,   .data_type = PANEL_COMM,	 .data = 0xB7},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0x75},
			{.delay = 0,   .data_type = PANEL_COMM,	 .data = 0xBB},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0x1f},//Vcom 0x1F: 0.875, 0x31: 1.35V
			{.delay = 0,   .data_type = PANEL_COMM,	 .data = 0xC0},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0x2C},
			{.delay = 0,   .data_type = PANEL_COMM,	 .data = 0xC2},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0x01},
			{.delay = 0,   .data_type = PANEL_COMM,	 .data = 0xC3},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0x13},
			{.delay = 0,   .data_type = PANEL_COMM,	 .data = 0xC4},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0x20},//VDV, 0x20:0v
			{.delay = 0,   .data_type = PANEL_COMM,	 .data = 0xC6},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0x0F},//Frame Rate control 0F: 60hz
			{.delay = 0,   .data_type = PANEL_COMM,	 .data = 0xD0},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0xA4},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0xA1},
			{.delay = 0,   .data_type = PANEL_COMM,	 .data = 0xD6},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0xA1},
			{.delay = 0,   .data_type = PANEL_COMM,	 .data = 0xE0},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0xD0},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0x0D},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0x16},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0x14},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0x14},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0x2F},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0x38},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0x54},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0x48},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0x0A},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0x1C},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0x19},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0x1A},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0x1D},
			{.delay = 0,   .data_type = PANEL_COMM,	 .data = 0xE1},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0xD0},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0x04},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0x0C},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0x08},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0x09},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0x23},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0x39},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0x54},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0x48},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0x30},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0x14},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0x14},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0x19},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0x1C},
			{.delay = 0,   .data_type = PANEL_COMM,	 .data = 0x2A},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0x00},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0x00},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0x00},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0xEF},  //240

			{.delay = 0,   .data_type = PANEL_COMM,	 .data = 0x2B},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0x00},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0x00},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0x01},
			{.delay = 0,   .data_type = PANEL_DATA,	 .data = 0x40},  //320

			{.delay = 0,   .data_type = PANEL_COMM,	 .data = 0x29},
			{.delay = 0,   .data_type = PANEL_COMM,	 .data = 0x2C},
		}
	},
	.sync_info = {
		.vid_hsa_pixels = 5,
		.vid_hbp_pixels = 20,
		.vid_hfp_pixels = 10,
		.vid_hline_pixels = 240,
		.vid_vsa_lines = 5,
		.vid_vbp_lines = 5,
		.vid_vfp_lines = 20,
		.vid_active_lines = 320,
		.vid_vsa_pos_polarity = true,
		.vid_hsa_pos_polarity = true,
	},
	.u16FrameRate = 60,
};

#endif // _MCU_PARAM_ST7789V3_H_