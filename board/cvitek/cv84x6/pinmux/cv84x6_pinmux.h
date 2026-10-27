/*
 * Copyright (C) Cvitek Co., Ltd. 2022-2022. All rights reserved.
 *
 * File Name: cv84x6_pinmux.h
 *
 * Description: Cvitek cv84x6 pinmux header
 */
#ifndef __CV84X6_PINMUX_H__
#define __CV84X6_PINMUX_H__

#include "./cv84x6_reg_G1_PINMUX_REG.h"
#include "./cv84x6_reg_G2_PINMUX_REG.h"
#include "./cv84x6_reg_G3_PINMUX_REG.h"
#include "./cv84x6_reg_G4_PINMUX_REG.h"
#include "./cv84x6_reg_G7_PINMUX_REG.h"
#include "./cv84x6_reg_G8_PINMUX_REG.h"
#include "./cv84x6_reg_PINMUX_VALUE.h"

/* pinmux register base */
#define PINMUX_BASE SYS_CTRL_BASE

/* pinmux register offset */
#define G1_PINMUX_REG_REG_BASE 0x28104100
#define G2_PINMUX_REG_REG_BASE 0x28104200
#define G3_PINMUX_REG_REG_BASE 0x28104300
#define G4_PINMUX_REG_REG_BASE 0x28104e00
#define G7_PINMUX_REG_REG_BASE 0x05027000
#define G8_PINMUX_REG_REG_BASE 0x05027800


#define PIN_PULL_DOWN 0x0
#define PIN_PULL_UP 0x1
#define PINMUX_INVALID_SEL 0xffff

typedef enum {
G1 = 1,
G2 = 2,
G3 = 3,
G4 = 4,
G7 = 7,
G8 = 8,
} group_pin_t;


#define PIN_MUX(PIN_NAME, GROUP, VALUE) \
({ \
    if (GROUP##_PINMUX_REG_REG_##PIN_NAME != PINMUX_INVALID_SEL) \
        mmio_clrsetbits_32(GROUP##_PINMUX_REG_REG_BASE + \
                        GROUP##_PINMUX_REG_REG_##PIN_NAME, \
                       GROUP##_PINMUX_REG_REG_##PIN_NAME##_PIN_SEL_EN_MASK, \
                       VALUE << GROUP##_PINMUX_REG_REG_##PIN_NAME##_PIN_SEL_EN_OFFSET); \
})

#define PINMUX_CONFIG(PIN_NAME, FUNC_NAME, GROUP) \
({ \
    PIN_MUX(PIN_NAME, GROUP, PINMUX_VAL__##PIN_NAME##__##FUNC_NAME); \
})


/*
bit2: Pull Up Enable for xxx 
bit3: Pull Down Enable for xxx 
*/
#define PIN_PULL_CONFIG(PIN_NAME, GROUP, UP_DOWN) \
({ \
    if (UP_DOWN) \
        mmio_clrsetbits_32(GROUP##_PINMUX_REG_REG_BASE + \
                       GROUP##_PINMUX_REG_REG_##PIN_NAME, \
                       GROUP##_PINMUX_REG_REG_##PIN_NAME##_PD_EN_MASK | \
                       GROUP##_PINMUX_REG_REG_##PIN_NAME##_PU_EN_MASK, \
                       0x1 << GROUP##_PINMUX_REG_REG_##PIN_NAME##_PU_EN_OFFSET); \
    else \
        mmio_clrsetbits_32(GROUP##_PINMUX_REG_REG_BASE + \
                       GROUP##_PINMUX_REG_REG_##PIN_NAME, \
                       GROUP##_PINMUX_REG_REG_##PIN_NAME##_PD_EN_MASK | \
                       GROUP##_PINMUX_REG_REG_##PIN_NAME##_PU_EN_MASK, \
                       0x1 << GROUP##_PINMUX_REG_REG_##PIN_NAME##_PD_EN_OFFSET); \
})

#endif /* __CV84X6_PINMUX_H__ */
