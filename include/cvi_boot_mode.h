/* SPDX-License-Identifier: GPL-2.0+ */
#ifndef __CVI_BOOT_MODE_H__
#define __CVI_BOOT_MODE_H__

#include "mmc.h"

/* Magic for the A/B struct when serialized. */
#define AVB_AB_MAGIC "\0AB0"
#define AVB_AB_MAGIC_LEN 4

/* Versioning for the on-disk A/B metadata - keep in sync with avbtool. */
#define AVB_AB_MAJOR_VERSION 1
#define AVB_AB_MINOR_VERSION 0

/* Size of AvbABData struct. */
#define AVB_AB_DATA_SIZE 32

/* Maximum values for slot data */
#define AVB_AB_MAX_PRIORITY 15
#define AVB_AB_MAX_TRIES_REMAINING 3

// RTC register
// if change this register address, be careful
// this Header file used by fsbl & u-boot & kernel & (build dts)
#define BOOT_MODE_REGISTER 0x05026ff0

#define REBOOT_FLAG             0x5242C300
/* normal boot */
#define BOOT_NORMAL             (REBOOT_FLAG + 0)
/* enter bootloader rockusb mode */
#define BOOT_BL_DOWNLOAD        (REBOOT_FLAG + 1)
/* enter recovery */
#define BOOT_RECOVERY           (REBOOT_FLAG + 3)
 /* enter fastboot mode */
#define BOOT_FASTBOOT           (REBOOT_FLAG + 5)
#define BOOT_CHARGING           (REBOOT_FLAG + 11)

enum android_boot_mode {
	ANDROID_BOOT_MODE_NORMAL = 0,
	ANDROID_BOOT_MODE_RECOVERY,
	ANDROID_BOOT_MODE_LOADER,
	ANDROID_BOOT_MODE_CHARGING,
	ANDROID_BOOT_MODE_UNDEFINE,
};

typedef enum {
	AVB_AB_FLOW_RESULT_OK,
	AVB_AB_FLOW_RESULT_OK_WITH_VERIFICATION_ERROR,
	AVB_AB_FLOW_RESULT_ERROR_OOM,
	AVB_AB_FLOW_RESULT_ERROR_IO,
	AVB_AB_FLOW_RESULT_ERROR_NO_BOOTABLE_SLOTS,
	AVB_AB_FLOW_RESULT_ERROR_INVALID_ARGUMENT
} AvbABFlowResult;
// The size of the entire misc partition is 4MB = 4 * 1024 * 1024
#define AVB_AB_OFFSET_IN_MISC 0 // The offset of AB information in the misc partition
#define BOOTLOADER_MESSAGE_OFFSET_IN_MISC 16 * 1024 // The offset of the bootloader information in the misc partition
#define UPDATE_MESSAGE_OFFSET_IN_MISC 1024 * 1024 // The offset of the update information in the misc partition

enum boot_mode_t {
	BOOT_MODE_A,
	BOOT_MODE_B,
	BOOT_MODE_R,
	BOOT_MODE_INVALID
};

struct boot_dev_info_t {
	struct mmc *mmc_host;
	struct cvsnfc_host *nand_host;
};

struct BootloaderMessage {
	char command[32];
	char status[32];
	char recovery[768];
	char needupdate[4];
	char systemFlag[252];
};

#pragma pack(4)

struct AvbABSlotData {
	uint8_t priority;
	uint8_t tries_remaining;
	uint8_t successful_boot;
	uint8_t reserved[1];
};

struct AvbABData {
	uint8_t magic[AVB_AB_MAGIC_LEN];
	uint8_t version_major;
	uint8_t version_minor;
	uint8_t reserved1[2];
	struct AvbABSlotData slots[2];
	uint8_t reserved2[12];
	uint32_t crc32;
};

#pragma pack()

struct mmc *get_mmcdevice(void);
void avb_ab_data_dump(void);
void get_addr_part_offset_size(void **addr, ulong *part_offset, ulong *part_size);
enum boot_mode_t get_cvi_boot_mode(void);

#endif /* __CVI_BOOT_MODE_H__ */
