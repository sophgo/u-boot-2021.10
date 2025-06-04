/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * (C) Copyright 2013
 * David Feng <fenghua@phytium.com.cn>
 *
 * (C) Copyright 2002
 * Gary Jennejohn, DENX Software Engineering, <garyj@denx.de>
 *
 * (C) Copyright 2010
 * Texas Instruments, <www.ti.com>
 *	Aneesh V <aneesh@ti.com>
 */
/*
MEMORY { .sram : ORIGIN = IMAGE_TEXT_BASE,
		LENGTH = IMAGE_MAX_SIZE }

MEMORY { .sdram : ORIGIN = CONFIG_SPL_BSS_START_ADDR,
		LENGTH = CONFIG_SPL_BSS_MAX_SIZE }
*/

OUTPUT_FORMAT("elf64-littleaarch64", "elf64-littleaarch64", "elf64-littleaarch64")
OUTPUT_ARCH(aarch64)
ENTRY(_start)
SECTIONS
{
	.text : {
		. = ALIGN(8);
		*(.__image_copy_start)
		CPUDIR/start.o (.text*)
		*(.text*)
	}

	.rodata : {
		. = ALIGN(8);
		*(SORT_BY_ALIGNMENT(SORT_BY_NAME(.rodata*)))
	}

	.data : {
		. = ALIGN(8);
		*(.data*)
	}

#ifdef CONFIG_SPL_RECOVER_DATA_SECTION
	.data_save : {
		*(.__data_save_start)
		. = SIZEOF(.data);
		*(.__data_save_end)
	}
#endif

	.u_boot_list : {
		. = ALIGN(8);
		KEEP(*(SORT(.u_boot_list*)));
	}

	.image_copy_end : {
		. = ALIGN(8);
		*(.__image_copy_end)
	}

	.end : {
		. = ALIGN(8);
		*(.__end)
	}

	_image_binary_end = .;

	.bss_start (NOLOAD) : {
		. = ALIGN(8);
		KEEP(*(.__bss_start));
	}

	.bss (NOLOAD) : {
		*(.bss*)
		 . = ALIGN(8);
	}

	.bss_end (NOLOAD) : {
		KEEP(*(.__bss_end));
	}
	__bss_size = __bss_end - __bss_start;

	/DISCARD/ : { *(.rela*) }
	/DISCARD/ : { *(.dynsym) }
	/DISCARD/ : { *(.dynstr*) }
	/DISCARD/ : { *(.dynamic*) }
	/DISCARD/ : { *(.plt*) }
	/DISCARD/ : { *(.interp*) }
	/DISCARD/ : { *(.gnu*) }
}
