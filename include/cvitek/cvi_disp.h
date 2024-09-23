/*
 * Copyright (C) Cvitek Co., Ltd. 2019-2020. All rights reserved.
 *
 */

#ifndef __CVI_DISP_H__
#define __CVI_DISP_H__

#include <dm.h>
#include <video.h>

#define VO_INTF_CVBS (0x01L << 0)
#define VO_INTF_YPBPR (0x01L << 1)
#define VO_INTF_VGA (0x01L << 2)
#define VO_INTF_BT656 (0x01L << 3)
#define VO_INTF_BT1120 (0x01L << 6)
#define VO_INTF_LCD (0x01L << 7)
#define VO_INTF_LCD_18BIT (0x01L << 10)
#define VO_INTF_LCD_24BIT (0x01L << 11)
#define VO_INTF_LCD_30BIT (0x01L << 12)
#define VO_INTF_MIPI (0x01L << 13)
#define VO_INTF_MIPI_SLAVE (0x01L << 14)
#define VO_INTF_HDMI (0x01L << 15)
#define VO_INTF_I80 (0x01L << 16)
#define VO_INTF_I80_HW (0x01L << 17)

#define MAX_VO_PINS 32

enum VO_TOP_SEL {
	VO_CLK_0 = 0,
	VO_CLK_1,
	VO_D0,
	VO_D1,
	VO_D2,
	VO_D3,
	VO_D4,
	VO_D5,
	VO_D6,
	VO_D7,
	VO_D8,
	VO_D9,
	VO_D10,
	VO_D11,
	VO_D12,
	VO_D13,
	VO_D14,
	VO_D15,
	VO_D16,
	VO_D17,
	VO_D18,
	VO_D19,
	VO_D20,
	VO_D21,
	VO_D22,
	VO_D23,
	VO_D24,
	VO_D25,
	VO_D26,
	VO_D27,
	VO_D_MAX,
};

enum VO_TOP_D_SEL {
	VO_VIVO_D0 = VO_D13,
	VO_VIVO_D1 = VO_D14,
	VO_VIVO_D2 = VO_D15,
	VO_VIVO_D3 = VO_D16,
	VO_VIVO_D4 = VO_D17,
	VO_VIVO_D5 = VO_D18,
	VO_VIVO_D6 = VO_D19,
	VO_VIVO_D7 = VO_D20,
	VO_VIVO_D8 = VO_D21,
	VO_VIVO_D9 = VO_D22,
	VO_VIVO_D10 = VO_D23,
	VO_VIVO_CLK = VO_CLK_1,
	VO_MIPI_TXM4 = VO_D24,
	VO_MIPI_TXP4 = VO_D25,
	VO_MIPI_TXM3 = VO_D26,
	VO_MIPI_TXP3 = VO_D27,
	VO_MIPI_TXM2 = VO_D0,
	VO_MIPI_TXP2 = VO_CLK_0,
	VO_MIPI_TXM1 = VO_D2,
	VO_MIPI_TXP1 = VO_D1,
	VO_MIPI_TXM0 = VO_D4,
	VO_MIPI_TXP0 = VO_D3,
	VO_MIPI_RXN5 = VO_D12,
	VO_MIPI_RXP5 = VO_D11,
	VO_MIPI_RXN2 = VO_D10,
	VO_MIPI_RXP2 = VO_D9,
	VO_MIPI_RXN1 = VO_D8,
	VO_MIPI_RXP1 = VO_D7,
	VO_MIPI_RXN0 = VO_D6,
	VO_MIPI_RXP0 = VO_D5,
	VO_PAD_MAX = VO_D_MAX
};

struct VO_D_REMAP {
	enum VO_TOP_D_SEL sel;
	u32 mux;
};

struct VO_PINMUX {
	unsigned char pin_num;
	struct VO_D_REMAP d_pins[MAX_VO_PINS];
};

struct sync_info_s {
	unsigned short  vid_hsa_pixels;
	unsigned short  vid_hbp_pixels;
	unsigned short  vid_hfp_pixels;
	unsigned short  vid_hline_pixels;
	unsigned short  vid_vsa_lines;
	unsigned short  vid_vbp_lines;
	unsigned short  vid_vfp_lines;
	unsigned short  vid_active_lines;
	unsigned short  edpi_cmd_size;
	bool            vid_vsa_pos_polarity;
	bool            vid_hsa_pos_polarity;
};

#endif // __CVI_DISP_H__
