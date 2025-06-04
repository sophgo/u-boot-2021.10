#ifndef _CVI_I80_HW_H_
#define _CVI_I80_HW_H_

#include "cvi_disp.h"

#define MAX_MCU_INSTR 256

typedef struct _HW_I80_INSTR_S {
	u8	delay;
	u8  data_type;
	u8	data;
} HW_I80_INSTR_S;

enum HW_I80_MODE {
	VO_MCU_MODE_RGB565 = 0,
	VO_MCU_MODE_RGB888,
	VO_MCU_MODE_MAX,
};

enum VO_TOP_I80_MUX {
	VO_MUX_MCU_CS = 0,
	VO_MUX_MCU_RS,
	VO_MUX_MCU_WR,
	VO_MUX_MCU_RD,
	VO_MUX_MCU_DATA0,
	VO_MUX_MCU_DATA1,
	VO_MUX_MCU_DATA2,
	VO_MUX_MCU_DATA3,
	VO_MUX_MCU_DATA4,
	VO_MUX_MCU_DATA5,
	VO_MUX_MCU_DATA6,
	VO_MUX_MCU_DATA7,
	VO_MUX_MAX,
};

struct HW_I80_INSTRS {
	unsigned char instr_num;
	HW_I80_INSTR_S instr_cmd[MAX_MCU_INSTR];
};

typedef struct _HW_I80_CFG_S {
	enum HW_I80_MODE mode;
	struct VO_PINMUX pins;
	struct HW_I80_INSTRS instrs;
	struct sync_info_s sync_info;
	unsigned short u16FrameRate;
} HW_I80_CFG_S;

int i80_hw_init(int VoDev, const HW_I80_CFG_S *i80_hw_cfg);

#endif // _CVI_I80_HW_H_