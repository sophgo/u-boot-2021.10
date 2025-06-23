#include <common.h>
#include <command.h>
#include <linux/mtd/mtd.h>
#include <nand.h>
/* malloc */
#include <malloc.h>
#include <env.h>
/* crc32 */
#include <u-boot/crc.h>
#include <cvi_update.h>
#include <cvi_repair_nand.h>

/**
 * progress bar
 * [=====================    ] 80%  IDX: idx(total)  RT: 10s
 */
static void print_progress_bar(int progress, int total, time_t total_time)
{
    int bar_width = 50;
    int percent = (100 * progress) / total;
    int pos = (bar_width * percent) / 100;

    /* remaining time */
    time_t remaining_time = total_time * (100 - percent) / 100; /* ms */

    /* print */
    printf("\r[");
    for (int i = 0; i < bar_width; i++) {
        if (i < pos) {
            printf("=");
        } else {
            printf(" ");
        }
    }
    printf("] %d%%  IDX: %d(%d)  RT: %lds    ", percent, progress, total,
           remaining_time / 1000);
}

/**
 * test a supposedly bad physical eraseblock.
 *   - good peb: return CVI_OK.
 *   - bad peb : return CVI_BAD_BLOCK.
 */
static TORTURE_STATUS cvi_torture_peb(struct mtd_info *mtd, loff_t offset)
{
    TORTURE_STATUS ret = CVI_OK;
    int check_ret;

    check_ret = nand_torture(mtd, offset);
    if (check_ret != 0) {
        ret = CVI_BAD_BLOCK;
    }

    return ret;
}

/**
 * Main repair nand flash cmd function.
 */
static int do_repair_nand(struct cmd_tbl *cmdtp, int flag, int argc,
                          char * const argv[])
{
    struct mtd_info *mtd = get_nand_dev_by_index(nand_curr_device);
    uint32_t block_count, block_index;
    loff_t offset;
    TORTURE_STATUS check_ret = CVI_OK;
    enum command_ret_t ret = CMD_RET_SUCCESS; /* fail: CMD_RET_FAILURE */
    time_t start_time, total_time = 0;

    block_count = mtd->size / mtd->erasesize;

    for (block_index = 0; block_index < block_count; ++ block_index) {
        /**
         * Progress bar: use the processing time of the first 2 to 9 blocks
         * to estimate the total time.
         */
        if (block_index == 2) {
            /* The first block has a bbt creation overhead */
            start_time = get_timer(0);
        }
        if (block_index == 10) {
            total_time = get_timer(start_time);
            total_time = total_time * block_count / 8;
        }
        print_progress_bar(block_index, block_count, total_time);

        offset = block_index * mtd->erasesize;

        check_ret = cvi_torture_peb(mtd, offset);
        if (check_ret == CVI_UNKNOW_ERROR) {
            printf("\nERROR: an unknown error occurred while torture_peb !\n");
            ret = CMD_RET_FAILURE;
            goto out;
        }

        if (check_ret == CVI_BAD_BLOCK) {
            printf("\nfind bad block in 0x%08llx, mark it.\n", offset);

            /* mark bad block */
            if (mtd_block_markbad(mtd, offset)) {
                printf("ERROR: block 0x%08llx NOT marked as bad!\n\n", offset);
                ret = CMD_RET_FAILURE;
            } else {
                printf("block 0x%08llx successfully marked as bad\n\n", offset);
            }
        }
    }

out:
    return ret;
}

U_BOOT_CMD(repair_nand, 1, 0, do_repair_nand,
    "repair_nand - scan the nand flash, identify unmarked bad blocks,\n"
    "and modify the bad block marking information in the oob area.\n",
    "repair_nand - scan the nand flash, identify unmarked bad blocks,\n"
    "and modify the bad block marking information in the oob area.\n");
