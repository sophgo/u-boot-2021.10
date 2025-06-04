/*
 * Copyright (C) Cvitek Co., Ltd. 2019-2021. All rights reserved.
 *
 */

#ifndef _CVI_LVDS_H_
#define _CVI_LVDS_H_

#include "cvi_disp.h"

#define LANE_MAX_NUM   5

enum VO_LVDS_LANE_ID {
	VO_LVDS_LANE_CLK = 0,
	VO_LVDS_LANE_0,
	VO_LVDS_LANE_1,
	VO_LVDS_LANE_2,
	VO_LVDS_LANE_3,
	VO_LVDS_LANE_MAX,
};

enum VO_LVDS_OUT_BIT_E {
	VO_LVDS_OUT_6BIT = 0,
	VO_LVDS_OUT_8BIT,
	VO_LVDS_OUT_10BIT,
	VO_LVDS_OUT_MAX,
};

enum VO_LVDS_MODE_E {
	VO_LVDS_MODE_JEIDA = 0,
	VO_LVDS_MODE_VESA,
	VO_LVDS_MODE_MAX,
};

typedef struct _VO_LVDS_ATTR_S {
	enum VO_LVDS_MODE_E lvds_vesa_mode;
	enum VO_LVDS_OUT_BIT_E out_bits;
	u8 chn_num;
	bool data_big_endian;
	enum VO_LVDS_LANE_ID lane_id[VO_LVDS_LANE_MAX];
	bool lane_pn_swap[VO_LVDS_LANE_MAX];
	struct sync_info_s      sync_info;
	unsigned short          u16FrameRate;
} VO_LVDS_ATTR_S;

int lvds_init(const VO_LVDS_ATTR_S *lvds_cfg);

#endif // _CVI_LVDS_H_
