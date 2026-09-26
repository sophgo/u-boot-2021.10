// SPDX-License-Identifier: BSD-2-Clause
/*
 * U-Boot command for Android A/B boot selection (CVI customized)
 * Usage: cvi_ab_select_slot <slot_var_name> <interface> <dev[:part|#part_name]>
 * Example: cvi_ab_select_slot boot_slot mmc 0:4  (Set boot_slot=a/b)
 * Example: cvi_ab_select_slot boot_slot mmc 0#misc (Find misc by partition name)
 */

#include <common.h>
#include <command.h>
#include <android_ab.h>
#include <android_bootloader_message.h>
#include <blk.h>
#include <part.h>
#include <malloc.h>
#include <memalign.h>
#include <linux/err.h>
#include <u-boot/crc.h>
#include <log.h>
#include <env.h>

/**
 * Compute CRC32 for bootloader control struct
 */
static uint32_t cvi_ab_control_compute_crc(struct bootloader_control *abc)
{
	return crc32(0, (void *)abc, offsetof(typeof(*abc), crc32_le));
}

/**
 * Reset bootloader control struct to default values
 */
static int cvi_ab_control_default(struct bootloader_control *abc)
{
	int i;
	const struct slot_metadata meta = {
	    .priority = 15,
	    .tries_remaining = 3,
	    .successful_boot = 0,
	    .verity_corrupted = 0,
	};

	if (!abc)
		return -EINVAL;

	/* Default to slot a */
	memcpy(abc->slot_suffix, "a\0\0\0", 4);
	abc->magic = BOOT_CTRL_MAGIC;
	abc->version = BOOT_CTRL_VERSION;
	abc->nb_slot = NUM_SLOTS;

	/* Initialize all slots with default metadata */
	for (i = 0; i < NUM_SLOTS; i++)
		abc->slot_info[i] = meta;

	/* Update CRC */
	abc->crc32_le = cvi_ab_control_compute_crc(abc);
	return 0;
}

/**
 * Save modified bootloader control struct back to disk
 */
static int cvi_ab_save_abc(struct blk_desc *dev, struct disk_partition *part,
			   struct bootloader_control *abc)
{
	ulong offset, blocks;

	/* Recalculate offset and blocks */
	offset = offsetof(struct bootloader_message_ab, slot_suffix) / part->blksz;
	blocks = DIV_ROUND_UP(sizeof(*abc), part->blksz);

	/* Update CRC before save */
	abc->crc32_le = cvi_ab_control_compute_crc(abc);

	/* Write back to disk */
	if (IS_ERR_VALUE(blk_dwrite(dev, part->start + offset, blocks, abc))) {
		log_err("CVI_AB: Failed to write AB control block\n");
		return -EIO;
	}

	return 0;
}

/**
 * Get current slot index from AB control struct (a=0, b=1...)
 */
static int cvi_ab_get_current_slot(struct bootloader_control *abc)
{
	char c = abc->slot_suffix[0];

	if (c >= 'a' && c < 'a' + abc->nb_slot)
		return c - 'a';

	log_warning("CVI_AB: Invalid slot suffix, default to 0 (a)\n");
	return 0;
}

/**
 * Set current slot in AB control struct (0=a, 1=b...)
 */
static void cvi_ab_set_current_slot(struct bootloader_control *abc, int slot)
{
	if (slot < 0 || slot >= abc->nb_slot) {
		log_err("CVI_AB: Invalid slot index %d, fallback to 0 (a)\n", slot);
		slot = 0; /* Force default to slot a */
	}

	memset(abc->slot_suffix, 0, 4);
	abc->slot_suffix[0] = 'a' + slot;
	log_info("CVI_AB: Set current slot to %c\n", 'a' + slot);
}

/**
 * Find next available slot (not corrupted and has remaining tries)
 * Return 0 (slot a) if no available slot found
 */
static int cvi_ab_find_next_available(struct bootloader_control *abc, int cur)
{
	int i, start = (cur + 1) % abc->nb_slot;

	/* Loop through slots starting from next one */
	for (i = start; i != cur; i = (i + 1) % abc->nb_slot) {
		if (!abc->slot_info[i].verity_corrupted &&
		    abc->slot_info[i].tries_remaining > 0) {
			log_info("CVI_AB: Found available slot %d (%c)\n", i, 'a' + i);
			return i;
		}
		log_debug("CVI_AB: Slot %d (%c) unavailable (corrupt: %d, tries: %d)\n",
			  i, 'a' + i, abc->slot_info[i].verity_corrupted,
			  abc->slot_info[i].tries_remaining);
	}

	/* Core change: return slot 0 (a) if no valid slot found */
	log_err("CVI_AB: No available slots left! Fallback to slot a (0)\n");
	return 0;
}

/**
 * Core logic for slot selection
 */
static int cvi_ab_do_select(struct blk_desc *dev_desc, struct disk_partition *part)
{
	struct bootloader_control *abc = NULL;
	uint32_t crc;
	int cur_slot, new_slot;
	bool need_save = false;
	ulong offset, blocks;

	/* Calculate AB control block offset (in blocks) */
	offset = offsetof(struct bootloader_message_ab, slot_suffix);
	if (offset % part->blksz) {
		log_err("CVI_AB: AB control block not aligned with block size\n");
		return 0; /* Alignment failed, default to slot a */
	}
	offset /= part->blksz;
	blocks = DIV_ROUND_UP(sizeof(struct bootloader_control), part->blksz);

	/* Check partition size */
	if (offset + blocks > part->size) {
		log_err("CVI_AB: Partition too small (need %lu blocks, have %lu)\n",
			offset + blocks, part->size);
		return 0; /* Partition too small, default to slot a */
	}

	/* Allocate aligned memory for AB control block */
	abc = malloc_cache_aligned(blocks * part->blksz);
	if (!abc) {
		log_err("CVI_AB: Out of memory\n");
		return 0; /* Out of memory, default to slot a */
	}

	/* Read AB control block from disk */
	if (IS_ERR_VALUE(blk_dread(dev_desc, part->start + offset, blocks, abc))) {
		log_err("CVI_AB: Failed to read AB control block\n");
		free(abc);
		return 0; /* Read failed, default to slot a */
	}

	/* Validate AB control block */
	crc = cvi_ab_control_compute_crc(abc);
	if (abc->magic != BOOT_CTRL_MAGIC || abc->crc32_le != crc) {
		log_warning("CVI_AB: AB control block invalid, reset to default (slot a)\n");
		cvi_ab_control_default(abc);
		need_save = true;
		cur_slot = 0; /* Invalid CRC, default to slot a */
	} else {
		/* Get current slot */
		cur_slot = cvi_ab_get_current_slot(abc);
		printf("CVI_AB: Current slot = %c\n", 'a' + cur_slot);

		/* Check if current slot is available */
		if (abc->slot_info[cur_slot].verity_corrupted ||
		    abc->slot_info[cur_slot].tries_remaining == 0) {
			printf("CVI_AB: Slot %c is unavailable, switching...\n", 'a' + cur_slot);
			goto switch_slot;
		}

		/* Clear successful_boot flag (kernel will set it if boot success) */
		if (abc->slot_info[cur_slot].successful_boot) {
			log_info("CVI_AB: Clear successful_boot flag for slot %c\n", 'a' + cur_slot);
			abc->slot_info[cur_slot].successful_boot = 0;
			need_save = true;
		}

		/* Decrement remaining tries */
		abc->slot_info[cur_slot].tries_remaining--;
		need_save = true;
		printf("CVI_AB: Slot %c tries remaining: %d\n",
		       'a' + cur_slot, abc->slot_info[cur_slot].tries_remaining);

		/* If no tries left, mark as corrupted and switch */
		if (abc->slot_info[cur_slot].tries_remaining == 0) {
			log_err("CVI_AB: Slot %c retry count exhausted, mark as corrupted\n", 'a' + cur_slot);
			abc->slot_info[cur_slot].verity_corrupted = 1;
			need_save = true;

switch_slot:
			/* Find next available slot (defaults to 0 = a if none) */
			new_slot = cvi_ab_find_next_available(abc, cur_slot);
			/* Force switch to new slot (even if it's default a) */
			cur_slot = new_slot;
			cvi_ab_set_current_slot(abc, cur_slot);
			abc->slot_info[cur_slot].successful_boot = 0;
			need_save = true;
		}
	}

	/* Save changes if needed */
	if (need_save)
		cvi_ab_save_abc(dev_desc, part, abc);

	/* Cleanup */
	free(abc);
	return cur_slot;
}

/**
 * U-Boot command handler for cvi_ab_select_slot
 * Based on native ab_select implementation, argument format:
 * <slot_var_name> <interface> <dev[:part|#part_name]>
 */
static int do_cvi_ab_select_slot(struct cmd_tbl *cmdtp, int flag,
				 int argc, char *const argv[])
{
	struct blk_desc *dev_desc;
	struct disk_partition part_info;
	int slot_idx;
	char slot[2]; /* Store slot character (a/b) */

	/* Check arguments: exactly 4 required (cmd + 3 params) */
	if (argc != 4) {
		printf("Usage: %s <slot_var_name> <interface> <dev[:part|#part_name]>\n", cmdtp->name);
		printf("Example: %s boot_slot mmc 0:4\n", cmdtp->name);
		printf("Example: %s boot_slot mmc 0#misc\n", cmdtp->name);
		return CMD_RET_USAGE;
	}

	/* Parse device and partition using native compatible helper */
	if (part_get_info_by_dev_and_name_or_num(argv[2], argv[3],
						 &dev_desc, &part_info,
						 false) < 0) {
		log_err("CVI_AB: Failed to find partition %s on %s\n", argv[3], argv[2]);
		/* If partition lookup fails, force slot a */
		log_err("CVI_AB: Fallback to slot a (0)\n");
		slot[0] = 'a';
		slot[1] = '\0';
		env_set(argv[1], slot);
		env_set("boot_slot_idx", "0");
		printf("CVI_AB: Booting slot: a (index: 0)\n");
		printf("CVI_AB: Env %s set to a / boot_slot_idx set to 0\n", argv[1]);
		return CMD_RET_SUCCESS; /* Return success, not failure */
	}

	/* Execute slot selection (all errors default to 0 = a) */
	slot_idx = cvi_ab_do_select(dev_desc, &part_info);

	/* Convert slot index to character (a/b) and set env variable */
	slot[0] = BOOT_SLOT_NAME(slot_idx); /* Use Android standard conversion */
	slot[1] = '\0';
	env_set(argv[1], slot);

	/* Optional: set numeric slot index for kernel usage */
	char slot_idx_str[8];

	sprintf(slot_idx_str, "%d", slot_idx);
	env_set("boot_slot_idx", slot_idx_str);

	/* Print result */
	printf("CVI_AB: Booting slot: %s (index: %d)\n", slot, slot_idx);
	printf("CVI_AB: Env %s set to %s / boot_slot_idx set to %s\n",
	       argv[1], slot, slot_idx_str);

	return CMD_RET_SUCCESS;
}

/**
 * U-Boot command table definition
 */
U_BOOT_CMD(cvi_ab_select_slot, 4, 0, do_cvi_ab_select_slot,
	   "Select CVI A/B boot slot and register boot attempt",
	   "<slot_var_name> <interface> <dev[:part|#part_name]>\n"
	   "    - Load slot metadata from specified partition, select active slot\n"
	   "      and store slot char (a/b) in <slot_var_name> env variable.\n"
	   "    - Fallback to slot a if no valid slot found.\n"
	   "    - Also set boot_slot_idx (num) for kernel to read.\n"
	   "    - Example: cvi_ab_select_slot boot_slot mmc 0:4\n"
	   "    - Example: cvi_ab_select_slot boot_slot mmc 0#misc\n");
