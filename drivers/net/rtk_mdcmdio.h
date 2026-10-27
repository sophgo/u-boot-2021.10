/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Minimal RTL83xx/RTL8367x ASIC register access over MDC/MDIO using U-Boot's
 * legacy miiphy API.
 *
 * This is intended to be portable to projects that only have:
 *   - miiphy_read()
 *   - miiphy_write()
 *
 * The ASIC register access sequence is derived from the Realtek SDK's
 * MDC/MDIO "reg_mdcmdio_*" implementation (PHYID=29, regs 31/23/21/24/25).
 */
 
 #ifndef __RTK_MDCMDIO_H__
 #define __RTK_MDCMDIO_H__
 
 #include <linux/types.h>
 
 /*
  * Realtek MDC/MDIO management PHY ID used by the SDK.
  * Some platforms use PHY ID 0 instead; keep it configurable.
  */
 #ifndef RTK_MDCMDIO_PHY_ID
 #define RTK_MDCMDIO_PHY_ID 29
 #endif
 
 /* Indirect access registers on the management PHY */
 #define RTK_MDCMDIO_CTRL0_REG		31
 #define RTK_MDCMDIO_CTRL1_REG		21
 #define RTK_MDCMDIO_ADDRESS_REG		23
 #define RTK_MDCMDIO_DATA_WRITE_REG	24
 #define RTK_MDCMDIO_DATA_READ_REG	25
 
 /* Control opcodes (written to CTRL0/CTRL1) */
 #define RTK_MDCMDIO_ADDR_OP		0x000E
 #define RTK_MDCMDIO_READ_OP		0x0001
 #define RTK_MDCMDIO_WRITE_OP		0x0003
 
 /* Common RTL8367x chip identification registers */
 #define RTK_RTL8367X_REG_CHIP_NUMBER	0x1300
 #define RTK_RTL8367X_REG_CHIP_VER	0x1301
 #define RTK_RTL8367X_REG_UNLOCK		0x13C2
 #define RTK_RTL8367X_UNLOCK_KEY		0x0249
 
 /*
  * Read/write a 16-bit RTL8367x ASIC register via the management PHY.
  *
  * devname: miiphy device name (usually the ethernet device name, e.g. "eth0")
  * reg:     ASIC register address (0..0xFFFF)
  * val:     16-bit value
  *
  * Return 0 on success, non-zero on failure (miiphy_* compatible).
  */
 int rtk_mdcmdio_read16(const char *devname, u16 reg, u16 *val);
 int rtk_mdcmdio_write16(const char *devname, u16 reg, u16 val);
 
 /*
  * Read RTL8367x chip number/version using the unlock sequence:
  *   write 0x13C2 = 0x0249
  *   read  0x1300 / 0x1301
  *   write 0x13C2 = 0x0000
  */
 int rtk_rtl8367x_read_chipid(const char *devname, u16 *chip_number, u16 *chip_ver);
 
 #endif /* __RTK_MDCMDIO_H__ */
 