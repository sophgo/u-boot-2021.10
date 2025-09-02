#ifndef __SPI_TUNING_H__
#define __SPI_TUNING_H__

/* tuning structure */
struct tuning_param_range {
	int min;
	int max;
};

struct tuning_ops {
	int param_num; // The number of param
	struct tuning_param_range param_ranges[2]; //  Assume up to 2 parameters
	int old_param[2];
};

#endif