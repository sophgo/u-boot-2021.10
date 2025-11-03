
#include <common.h>
#include <command.h>
#include <tempsen.h>


int do_chiptemp(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	static char init_flag = 0;
	if (init_flag == 0) {
		 tempsen_init();
		 printf("Temperature sensor init!\n");
		 init_flag = 1;
	}
    printf("Temperature(m℃):%d\n", read_temp());

    return CMD_RET_SUCCESS;
}


U_BOOT_CMD(
	chiptemp,	1,	1,	do_chiptemp,
	"Detect chip temperature, unit mC",
	"- Return the chip temperature, unit mC"
);


