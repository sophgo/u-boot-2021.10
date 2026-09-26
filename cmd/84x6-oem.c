// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2024 Sophgo
 *
 * OEM information read/write command for cv84x6/cv186x platform.
 * OEM data is stored on eMMC boot1 partition (block 0, offset 0).
 */

#include <common.h>
#include <command.h>
#include <stdlib.h>
#include <mmio.h>
#include <linux/ctype.h>

/* DRAM buffer for eMMC boot1 read/write operations */
#define EMMC_LOAD_ADDR		0x1004000000ULL
#define OEM_TOTAL_SIZE		256

struct oem_field {
	const char *name;	/* field name for command lookup */
	uint32_t offset;	/* byte offset from OEM base */
	uint32_t size;		/* field size in bytes */
	const char *desc;	/* human-readable description */
};

/*
 * OEM field layout matching the specification table.
 * Total 256 bytes, offsets 0x00 ~ 0xF0.
 */
static const struct oem_field oem_fields[] = {
	{ "SN0",           0x00, 32, "chip sn" },
	{ "SN1",           0x20, 32, "reserve for product or customer" },
	{ "MAC0",          0x40, 16, "mac0" },
	{ "MAC1",          0x50, 16, "mac1" },
	{ "PRODUCT_TYPE",  0x60, 16, "product type" },
	{ "MODULE_TYPE",   0x70, 16, "module type" },
	{ "INTERFACE_FLAG",0x80,  1, "factory interface test flag" },
	{ "AGING_FLAG",    0x81,  1, "factory aging test flag" },
	{ "CONSOLE_FLAG",  0x82,  1, "select uart flag for console" },
	{ "PHY_LED_FLAG",  0x83,  1, "select eth phy led type" },
	{ "VENDER",        0x90, 16, "vender" },
	{ "DTS_TYPE",      0xa0, 32, "dts type" },
	{ "HW_VERSION",    0xc0, 16, "hw version" },
	{ "PRODUCT",       0xd0, 16, "product" },
	{ "CHIP",          0xe0, 16, "chip (BM1688/CV186AH)" },
	{ "DDR_SIZE",      0xf0,  1, "ddr total size (HEX GB)" },
};

static int find_oem_field(const char *name)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(oem_fields); i++) {
		if (!strcasecmp(name, oem_fields[i].name))
			return i;
	}
	return -1;
}

/*
 * Read a byte from the eMMC boot1 OEM buffer (already loaded to DRAM).
 */
static inline uint8_t oem_buf_read_byte(uint32_t offset)
{
	return mmio_read_8(EMMC_LOAD_ADDR + offset);
}

/*
 * Write a byte to the eMMC boot1 OEM buffer (in DRAM).
 */
static inline void oem_buf_write_byte(uint32_t offset, uint8_t val)
{
	mmio_write_8(EMMC_LOAD_ADDR + offset, val);
}

/*
 * Load OEM data from eMMC boot1 into DRAM buffer.
 * Switches to eMMC boot1 partition and reads block 0.
 */
static int oem_load_from_emmc(void)
{
	run_command("mmc dev 0 2", 0);
	run_command("mmc read 0x1004000000 0 1", 0);
	return 0;
}

/*
 * Write OEM data from DRAM buffer back to eMMC boot1.
 * Switches to eMMC boot1 partition and writes block 0.
 */
static int oem_save_to_emmc(void)
{
	run_command("mmc dev 0 2", 0);
	run_command("mmc write 0x1004000000 0 1", 0);
	return 0;
}

/*
 * Convert hex string to bytes. Each pair of hex characters becomes one byte.
 * Returns number of bytes converted.
 */
static int hex2bytes(const char *hex, unsigned char *buf, int buf_size)
{
	int i, total = 0;
	char tmp[3];

	memset(buf, 0, buf_size);

	for (i = 0; i < buf_size; i++) {
		if (!hex[0] || !hex[1])
			break;

		tmp[0] = hex[0];
		tmp[1] = hex[1];
		tmp[2] = '\0';

		buf[i] = simple_strtoul(tmp, NULL, 16);
		hex += 2;
		total += 1;
	}

	return total;
}

/*
 * Check if a string is all printable ASCII characters.
 */
static int is_printable_string(const unsigned char *buf, int len)
{
	int i;
	int has_content = 0;

	for (i = 0; i < len; i++) {
		if (buf[i] == '\0')
			break;
		if (buf[i] < 0x20 || buf[i] > 0x7e)
			return 0;
		has_content = 1;
	}
	return has_content;
}

/*
 * Display a field value. Handles string fields and byte fields.
 */
static void print_field(const struct oem_field *field)
{
	unsigned char buf[64] = {0};
	int i;

	/* Read field data from eMMC DRAM buffer */
	for (i = 0; i < field->size; i++)
		buf[i] = oem_buf_read_byte(field->offset + i);

	printf("%-16s ", field->name);

	/*
	 * For multi-byte fields that contain printable strings,
	 * display as string. For single-byte fields, show as hex.
	 */
	if (field->size == 1) {
		printf("0x%02X", buf[0]);
	} else if (is_printable_string(buf, field->size)) {
		printf("\"%-*s\"", field->size, buf);
	} else {
		/* Display as hex dump */
		printf("[");
		for (i = 0; i < field->size; i++) {
			if (i > 0 && (i % 16) == 0)
				printf("\n%16s ", "");
			printf("%02X", buf[i]);
			if (i < field->size - 1 && (i + 1) % 16 != 0)
				printf(" ");
		}
		printf("]");
	}

	printf("  (%s)\n", field->desc);
}

/*
 * Display all OEM fields from eMMC boot1.
 */
static int do_oem_read_all(void)
{
	int i;

	/* Load from eMMC boot1 to DRAM buffer */
	oem_load_from_emmc();

	printf("\nOEM Information (eMMC boot1):\n");
	printf("======================================\n");
	for (i = 0; i < ARRAY_SIZE(oem_fields); i++)
		print_field(&oem_fields[i]);
	printf("======================================\n");

	return CMD_RET_SUCCESS;
}

/*
 * Display a specific OEM field with detailed hex dump from eMMC boot1.
 */
static int do_oem_read_field(const char *name)
{
	int idx;
	int i;
	unsigned char buf[64] = {0};

	idx = find_oem_field(name);
	if (idx < 0) {
		printf("Unknown OEM field: %s\n", name);
		printf("Available fields:\n");
		for (i = 0; i < ARRAY_SIZE(oem_fields); i++)
			printf("  %s\n", oem_fields[i].name);
		return CMD_RET_USAGE;
	}

	/* Load from eMMC boot1 to DRAM buffer */
	oem_load_from_emmc();

	for (i = 0; i < oem_fields[idx].size; i++)
		buf[i] = oem_buf_read_byte(oem_fields[idx].offset + i);

	printf("Field: %s (%s)\n", oem_fields[idx].name, oem_fields[idx].desc);
	printf("Offset: 0x%02X  Size: %d bytes\n",
	       oem_fields[idx].offset, oem_fields[idx].size);
	printf("Value (hex): ");
	print_buffer(0, buf, 1, oem_fields[idx].size, 0);

	if (is_printable_string(buf, oem_fields[idx].size)) {
		char tmp[65] = {0};
		memcpy(tmp, buf, min_t(int, oem_fields[idx].size, 64));
		printf("Value (str): \"%s\"\n", tmp);
	}

	return CMD_RET_SUCCESS;
}

/*
 * Write a value to an OEM field directly to eMMC boot1.
 */
static int do_oem_write(const char *name, const char *hex_val)
{
	int idx;
	unsigned char buf[64] = {0};
	int bytes_written;
	int i;

	idx = find_oem_field(name);
	if (idx < 0) {
		printf("Unknown OEM field: %s\n", name);
		return CMD_RET_USAGE;
	}

	bytes_written = hex2bytes(hex_val, buf, oem_fields[idx].size);
	if (bytes_written <= 0) {
		printf("Invalid hex value: %s\n", hex_val);
		return CMD_RET_USAGE;
	}

	printf("Writing %s: ", oem_fields[idx].name);
	for (i = 0; i < bytes_written; i++)
		printf("%02X", buf[i]);
	printf("\n");

	/* Load current OEM data from eMMC boot1 */
	oem_load_from_emmc();

	/* Modify the field data in DRAM buffer */
	for (i = 0; i < oem_fields[idx].size; i++) {
		if (i < bytes_written)
			oem_buf_write_byte(oem_fields[idx].offset + i, buf[i]);
		else
			oem_buf_write_byte(oem_fields[idx].offset + i, 0);
	}

	/* Write back to eMMC boot1 */
	oem_save_to_emmc();

	/* Verify by reading back */
	oem_load_from_emmc();
	printf("Verify: ");
	for (i = 0; i < oem_fields[idx].size; i++)
		printf("%02X", oem_buf_read_byte(oem_fields[idx].offset + i));
	printf("\n");

	printf("OEM data written to eMMC boot1.\n");

	return CMD_RET_SUCCESS;
}

/*
 * Display the list of OEM fields with current values from eMMC boot1.
 */
static void format_field_value(const struct oem_field *field, char *out, int out_size)
{
	unsigned char buf[64] = {0};
	int i;

	for (i = 0; i < field->size; i++)
		buf[i] = oem_buf_read_byte(field->offset + i);

	if (field->size == 1) {
		snprintf(out, out_size, "0x%02X", buf[0]);
	} else if (is_printable_string(buf, field->size)) {
		/* Truncate string display for table format */
		memcpy(out, buf, min_t(int, field->size, out_size - 1));
		out[min_t(int, field->size, out_size - 1)] = '\0';
	} else {
		/* Hex dump for non-printable fields */
		int pos = 0;
		for (i = 0; i < field->size && pos < out_size - 3; i++) {
			pos += snprintf(out + pos, out_size - pos, "%02X", buf[i]);
		}
	}
}

static int do_oem_list(void)
{
	int i;

	/* Load from eMMC boot1 */
	oem_load_from_emmc();

	printf("\nOEM Fields (eMMC boot1):\n");
	printf("%-16s %6s %4s  %-32s  %s\n",
	       "Name", "Offset", "Size", "Value", "Description");
	printf("%-16s %6s %4s  %-32s  %s\n",
	       "----", "------", "----", "-----", "-----------");

	for (i = 0; i < ARRAY_SIZE(oem_fields); i++) {
		char val_str[64] = {0};

		format_field_value(&oem_fields[i], val_str, sizeof(val_str));
		printf("%-16s 0x%04X %4d  %-32s  %s\n",
		       oem_fields[i].name,
		       oem_fields[i].offset,
		       oem_fields[i].size,
		       val_str,
		       oem_fields[i].desc);
	}

	printf("\nTotal: %d fields, %d bytes\n",
	       (int)ARRAY_SIZE(oem_fields), OEM_TOTAL_SIZE);

	return CMD_RET_SUCCESS;
}

/* ================================================================
 * Subcommand handlers (dispatched by U_BOOT_CMD_WITH_SUBCMDS)
 * argv[0] = subcommand name
 * ================================================================ */

/*
 * Handle "oem read" subcommand
 * argc=1: "oem read" -> display all
 * argc=2: "oem read <field>" -> display specific field
 */
static int do_oem_read(struct cmd_tbl *cmdtp, int flag, int argc,
		       char *const argv[])
{
	if (argc == 1)
		return do_oem_read_all();
	else
		return do_oem_read_field(argv[1]);
}

/*
 * Handle "oem write" subcommand
 * Usage: oem write <field> <hex_value>
 * argc=3: argv[0]="write", argv[1]=field, argv[2]=hex_value
 */
static int do_oem_write_cmd(struct cmd_tbl *cmdtp, int flag, int argc,
			    char *const argv[])
{
	if (argc != 3)
		return CMD_RET_USAGE;

	return do_oem_write(argv[1], argv[2]);
}

/*
 * Handle "oem list" subcommand
 * argc=1: argv[0]="list"
 */
static int do_oem_list_cmd(struct cmd_tbl *cmdtp, int flag, int argc,
			   char *const argv[])
{
	if (argc != 1)
		return CMD_RET_USAGE;

	return do_oem_list();
}

static char oem_help_text[] =
	"oem read                       - display all OEM fields from eMMC boot1\n"
	"oem read  <field>              - display a specific OEM field\n"
	"oem write <field> <hex_value>  - write hex value to field (directly to eMMC boot1)\n"
	"oem list                       - list all OEM fields and descriptions\n"
	"\n"
	"Fields: SN0 SN1 MAC0 MAC1 PRODUCT_TYPE MODULE_TYPE\n"
	"        INTERFACE_FLAG AGING_FLAG CONSOLE_FLAG PHY_LED_FLAG\n"
	"        VENDER DTS_TYPE HW_VERSION PRODUCT CHIP DDR_SIZE";

U_BOOT_CMD_WITH_SUBCMDS(oem, "OEM information", oem_help_text,
	U_BOOT_SUBCMD_MKENT(read,  2, 0, do_oem_read),
	U_BOOT_SUBCMD_MKENT(write, 3, 0, do_oem_write_cmd),
	U_BOOT_SUBCMD_MKENT(list,  1, 0, do_oem_list_cmd));
