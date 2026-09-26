// SPDX-License-Identifier: GPL-2.0+
/*
 * Minimal RTL8367x ASIC register access over MDC/MDIO using U-Boot miiphy API.
 *
 * This file is intentionally self-contained and portable.
 */
 
 #include "rtk_mdcmdio.h"

 #include <errno.h>
 #include <miiphy.h>
 
 static int rtk_miiphy_read_u16(const char *devname, u8 phyid, u8 reg, u16 *val)
 {
     unsigned short tmp = 0xffff;
     int ret;
 
     if (!val)
         return -EINVAL;
 
     ret = miiphy_read(devname, phyid, reg, &tmp);
     if (ret)
         return ret;
 
     *val = (u16)tmp;
     return 0;
 }
 
 static int rtk_miiphy_write_u16(const char *devname, u8 phyid, u8 reg, u16 val)
 {
     return miiphy_write(devname, phyid, reg, val);
 }
 
 int rtk_mdcmdio_read16(const char *devname, u16 reg, u16 *val)
 {
     int ret;
     u16 tmp;
 
     if (!devname)
         return -ENODEV;
     if (!val)
         return -EINVAL;
 
     /* Write address control code to reg 31 */
     ret = rtk_miiphy_write_u16(devname, RTK_MDCMDIO_PHY_ID,
                    RTK_MDCMDIO_CTRL0_REG, RTK_MDCMDIO_ADDR_OP);
     if (ret)
         return ret;
 
     /* Write target ASIC register address to reg 23 */
     ret = rtk_miiphy_write_u16(devname, RTK_MDCMDIO_PHY_ID,
                    RTK_MDCMDIO_ADDRESS_REG, reg);
     if (ret)
         return ret;
 
     /* Trigger read operation via reg 21 */
     ret = rtk_miiphy_write_u16(devname, RTK_MDCMDIO_PHY_ID,
                    RTK_MDCMDIO_CTRL1_REG, RTK_MDCMDIO_READ_OP);
     if (ret)
         return ret;
 
     /* Read data back from reg 25 */
     ret = rtk_miiphy_read_u16(devname, RTK_MDCMDIO_PHY_ID,
                   RTK_MDCMDIO_DATA_READ_REG, &tmp);
     if (ret)
         return ret;
 
     *val = tmp;
     return 0;
 }
 
 int rtk_mdcmdio_write16(const char *devname, u16 reg, u16 val)
 {
     int ret;
 
     if (!devname)
         return -ENODEV;
 
     /* Write address control code to reg 31 */
     ret = rtk_miiphy_write_u16(devname, RTK_MDCMDIO_PHY_ID,
                    RTK_MDCMDIO_CTRL0_REG, RTK_MDCMDIO_ADDR_OP);
     if (ret)
         return ret;
 
     /* Write target ASIC register address to reg 23 */
     ret = rtk_miiphy_write_u16(devname, RTK_MDCMDIO_PHY_ID,
                    RTK_MDCMDIO_ADDRESS_REG, reg);
     if (ret)
         return ret;
 
     /* Write data to reg 24 */
     ret = rtk_miiphy_write_u16(devname, RTK_MDCMDIO_PHY_ID,
                    RTK_MDCMDIO_DATA_WRITE_REG, val);
     if (ret)
         return ret;
 
     /* Trigger write operation via reg 21 */
     ret = rtk_miiphy_write_u16(devname, RTK_MDCMDIO_PHY_ID,
                    RTK_MDCMDIO_CTRL1_REG, RTK_MDCMDIO_WRITE_OP);
     if (ret)
         return ret;
 
     return 0;
 }
 
 int rtk_rtl8367x_read_chipid(const char *devname, u16 *chip_number, u16 *chip_ver)
 {
     int ret;
     u16 num = 0, ver = 0;
 
     if (!devname)
         return -ENODEV;
     if (!chip_number || !chip_ver)
         return -EINVAL;
 
     ret = rtk_mdcmdio_write16(devname, RTK_RTL8367X_REG_UNLOCK,
                   RTK_RTL8367X_UNLOCK_KEY);
     if (ret)
         return ret;
 
     ret = rtk_mdcmdio_read16(devname, RTK_RTL8367X_REG_CHIP_NUMBER, &num);
     if (ret)
         goto out_lock;
 
     ret = rtk_mdcmdio_read16(devname, RTK_RTL8367X_REG_CHIP_VER, &ver);
     if (ret)
         goto out_lock;
 
     *chip_number = num;
     *chip_ver = ver;
 
 out_lock:
     /* Best-effort lock back */
     (void)rtk_mdcmdio_write16(devname, RTK_RTL8367X_REG_UNLOCK, 0x0000);
     return ret;
 }
 
 