#ifndef CONFIG_SPL_BUILD
#include <common.h>
#include <command.h>
#include <fdt_support.h>
#include <image.h>
#include <linux/libfdt.h>
#include <spi_flash.h>
#include <dm/device.h>

static int update_ion_size(ulong fit_addr, uint64_t ion_size);

static int do_set_ion_size(struct cmd_tbl *cmdtp, int flag,
                      int argc, char *const argv[])
{
    int ret;
    uint64_t ion_size;
    ulong fit_addr;
    if (argc < 2) {
        printf("Usage: auto_set <ion_size>\n");
        printf("Example: auto_set 0x3b00000\n");
        return CMD_RET_USAGE;
    }

    #if defined(CONFIG_NAND_SUPPORT)
        ret = env_set("boot_device", "nand");
    #elif defined(CONFIG_SPI_FLASH)
        ret = env_set("boot_device", "nor");
    #else
        ret = env_set("boot_device", "emmc");
    #endif

    if (ret) {
        printf("Error: Failed to set boot_device\n");
        return CMD_RET_FAILURE;
    }

    ret = run_command("run loadcmd", 0);
    if (ret) {
        printf("Error: loadcmd failed\n");
        return CMD_RET_FAILURE;
    }
    fit_addr = env_get_ulong("uImage_addr", 16, CONFIG_SYS_LOAD_ADDR);
    ion_size = simple_strtoull(argv[1], NULL, 16);

    ret = update_ion_size(fit_addr, ion_size);
    if (ret != CMD_RET_SUCCESS) {
        return ret;
    }

    const char *updateion = env_get("updateion");
    if (updateion && (updateion[0] == 'y' || updateion[0] == 'Y')) {
        ret = run_command("run flash_fdt_cmd", 0);
        if (ret) {
            printf("Error: flash_fdt_cmd failed\n");
            return CMD_RET_FAILURE;
        }
    }

    return CMD_RET_SUCCESS;
}

static int update_ion_size(ulong fit_addr, uint64_t ion_size)
{
    void *fit_fdt, *fdt;
    int ret, nodeoff, len, images_noffset, fdt_noffset;
    const char *board_name;
    char fdt_node_name[256];
    const fdt32_t *old_ranges;
    fdt32_t alloc_ranges[4];
    fdt32_t size[2];
    const void *fdt_data;
    int fdt_len;

    board_name = env_get("board_name");
    if (!board_name) {
        printf("Error: board_name not set in environment\n");
        return CMD_RET_FAILURE;
    }
    fit_fdt = (void *)fit_addr;
    if (fdt_check_header(fit_fdt)) {
        printf("Error: Invalid FIT image at 0x%lx\n", fit_addr);
        return CMD_RET_FAILURE;
    }

    images_noffset = fdt_path_offset(fit_fdt, "/images");
    if (images_noffset < 0) {
        printf("Error: Cannot find /images node in FIT\n");
        return CMD_RET_FAILURE;
    }

    snprintf(fdt_node_name, sizeof(fdt_node_name), "fdt-%s", board_name);
    printf("Looking for FDT node: %s\n", fdt_node_name);

    fdt_noffset = fdt_subnode_offset(fit_fdt, images_noffset, fdt_node_name);
    if (fdt_noffset < 0) {
        printf("Error: Cannot find %s node in FIT\n", fdt_node_name);
        return CMD_RET_FAILURE;
    }

    fdt_data = fdt_getprop(fit_fdt, fdt_noffset, "data", &fdt_len);
    if (!fdt_data || fdt_len <= 0) {
        printf("Error: Cannot get data property from FDT node\n");
        return CMD_RET_FAILURE;
    }

    fdt = (void *)fdt_data;
    if (fdt_check_header(fdt)) {
        printf("Error: Invalid FDT in data property\n");
        return CMD_RET_FAILURE;
    }

    nodeoff = fdt_path_offset(fdt, "/reserved-memory/ion");
    if (nodeoff < 0) {
        printf("Error: Cannot find /reserved-memory/ion node\n");
        return CMD_RET_FAILURE;
    }

    old_ranges = fdt_getprop(fdt, nodeoff, "alloc-ranges", &len);
    if (!old_ranges) {
        printf("Error: Cannot find alloc-ranges property\n");
        return CMD_RET_FAILURE;
    }
    printf("alloc-ranges property length: %d bytes\n", len);

    if (len != 16) {
        printf("Error: Unexpected alloc-ranges size: %d (expected 16)\n", len);
        return CMD_RET_FAILURE;
    }

    alloc_ranges[0] = old_ranges[0];
    alloc_ranges[1] = old_ranges[1];
    alloc_ranges[2] = cpu_to_fdt32(0);
    alloc_ranges[3] = cpu_to_fdt32(ion_size);

    ret = fdt_setprop(fdt, nodeoff, "alloc-ranges",
                      alloc_ranges, sizeof(alloc_ranges));
    if (ret) {
        printf("Error: Cannot set ion alloc-ranges\n");
        return CMD_RET_FAILURE;
    }

    size[0] = cpu_to_fdt32(0);
    size[1] = cpu_to_fdt32(ion_size);

    ret = fdt_setprop(fdt, nodeoff, "size",
                      size, sizeof(size));
    if (ret) {
        printf("Error: Cannot set ion size\n");
        return CMD_RET_FAILURE;
    }

    printf("Successfully updated ion memory size to 0x%llx in FDT at 0x%p\n",
           ion_size, fdt);

    return CMD_RET_SUCCESS;
}

U_BOOT_CMD(
    set_ion_size, 2, 1, do_set_ion_size,
    "Set ION memory size in device tree,if setenv updateion y, will flash fdt after set ion size",
    "<ion_size>\n"
    "  - ion_size: New ION memory size (hex)\n"
    "Example:\n"
    "  set_ion_size  0x3b00000"
);

#endif