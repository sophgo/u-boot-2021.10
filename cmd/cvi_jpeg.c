#include <stdlib.h>
#include <common.h>
#include <command.h>
#include <linux/kconfig.h>
#include <cvi_disp.h>

extern int jpeg_decoder(void *bs_addr, void *yuv_addr, int size, int rotation);
extern int get_jpeg_size(int *width_addr, int *height_addr);

static int get_uboot_rotation(void)
{
#if IS_ENABLED(CONFIG_VIDEO_CVITEK)
	return cvi_disp_get_uboot_rotation();
#else
	return 0;
#endif
}

static int do_cvi_jpeg_dec(struct cmd_tbl *cmdtp, int flag, int argc,
			   char *const argv[])
{
	char *bs_addr = NULL;
	char *yuv_addr = NULL;
	int size = 0;
	int rotation = 0;
	char *endp = NULL;

	if (argc != 4 && argc != 5) {
		printf("Usage:\n%s\n", cmdtp->usage);
		return 1;
	}

	bs_addr = (char *)simple_strtol(argv[1], NULL, 16);

	if (!bs_addr) {
		printf("Usage:\n%s\n", cmdtp->usage);
		return 1;
	}

	yuv_addr = (char *)simple_strtol(argv[2], NULL, 16);

	if (!yuv_addr) {
		printf("Usage:\n%s\n", cmdtp->usage);
		return 1;
	}

	size = (int)simple_strtol(argv[3], NULL, 16);

	if (!size) {
		printf("Usage:\n%s\n", cmdtp->usage);
		return 1;
	}

	rotation = get_uboot_rotation();
	if (argc == 5) {
		rotation = simple_strtol(argv[4], &endp, 10);
		if (*argv[4] == 0 || *endp != 0) {
			printf("Usage:\n%s\n", cmdtp->usage);
			return 1;
		}
		if (!cvi_disp_is_valid_rotation(rotation)) {
			printf("rotation must be one of 0/90/180/270\n");
			return 1;
		}
	}

	printf("\nstart jpeg dec task!, bs_addr %p, yuv_addr %p, size %d, rotation %d\n",
	       bs_addr, yuv_addr, size, rotation);

	jpeg_decoder(bs_addr, yuv_addr, size, rotation);
	get_jpeg_size((int *)(bs_addr + size - 8), (int *)(bs_addr + size - 4));

	return CMD_RET_SUCCESS;
}

U_BOOT_CMD(
	cvi_jpeg_dec, 5, 0, do_cvi_jpeg_dec, "Jpeg decoder ",
	"cvi_jpeg_dec <bs_addr> <yuv_addr> <size> [rotation]\n"
	"rotation: optional, 0/90/180/270, default from cvitek,vo/uboot-rotation");
