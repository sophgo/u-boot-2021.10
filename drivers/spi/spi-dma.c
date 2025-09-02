/*
 * Copyright (C) 2018 bitmain
 */
#include <common.h>
#include <clk.h>
#include "spi-dma.h"
#include "mmio.h"
#include <asm/cache.h>
#include <cpu_func.h>
#include <linux/delay.h>
u64 *dma_addr_prev;

u64 *tx_lli_start_addr;
u64 *rx_lli_start_addr;

u64 *lli_malloc_addr;

void dma_turn_off(void)
{
	DMAC_WRITE(0x10, 0x0);
}

void dma_turn_on(void)
{
	DMAC_WRITE(0x10, 0x3);
}

void dma_reset(void)
{
	printf("%s\n", __func__);
	DMAC_WRITE(0x58, 0x1);
}

void dma_mem2dev_setting(unsigned int *src_addr, unsigned int len, unsigned int *reg, unsigned int p_dev)
{
	u64 ctl;
	unsigned int first_lli = 1;
	u64 *llp_addr;
	size_t offset;
	size_t xfer_count;
	unsigned int trans_width = 0x2;
	unsigned int i = 0;

/*
2.1 prepare work, default ctl setting
	prepare ctl reg
	source and dest data width, tr_width
	data width
	reg_width 0x0:bits,0x1:16bits
	src and dst mszie 0x4 32 data item, 32bits per item data
	SMS and DMS
	src inc and dst no change when dma do mem2dev copy
*/
//	DMAC_WRITE(0x10, 0x0);	//turn off dma first

	switch (p_dev) {
	case DMA_I2S0:
	case DMA_I2S1:
	case DMA_I2S2:
	case DMA_I2S3:
	case DMA_I2S4:
	case DMA_I2S5:
	case DMA_DW_I2S:
		//trans_width=0x2;
		ctl = 0x3 << 14 //SRC MSIZE
			| 0x3 << 18 //DST MSIZE
			| 0x2 << 8  //src_tr_width
			| 0x2 << 11 //dst_tr_width
			| 0x0UL << 48
			| 0x1UL << 47
			| 0xFUL << 39
			| 0x1UL << 38;
		break;
	case DMA_I2C0:
	case DMA_I2C1:
	case DMA_I2C2:
	case DMA_I2C3:
	case DMA_I2C4:
		//trans_width=0x1;
		ctl = 0x3 << 14 //SRC MSIZE
			| 0x3 << 18 //DST MSIZE
			| 0x2 << 8  //src_tr_width
			| 0x2 << 11; //dst_tr_width
		break;
	case DMA_SPI0:
		ctl = 0x1 << 14 //SRC MSIZE
			| 0x1 << 18 //DST MSIZE
			| 0x2 << 8  //src_tr_width
			| 0x2 << 11; //dst_tr_width
		break;
	default:
		return;
	}

	ctl |= 0x0 << 0 //SMS
		| 0x0 << 2 //DMS
		| 0X0 << 4 //SRC INC
		| 0X1 << 6; //DST no change

/*
2.2 prepare link list setting
	len: to send data size
	xfer_count: the count of link list items
	trans_width: data_width,32bits fixed
	src_addr, dst_addr: memory addr
	max block ts
	trans_width = 0x2
*/
	int lli_num = (len + 64 - 1) / 64 + 1;

	for (offset = 0; offset < len; offset = (offset + (xfer_count << trans_width))) {
		xfer_count = min_t(size_t, (len - offset) >> trans_width, 16);
		if (xfer_count == 0)
			xfer_count = 1;
		//Be make sure the allocated address with mask ~0x3f shouldn't re-write some meaningful memory
		if (!tx_lli_start_addr) {
#ifndef CONFIG_SPL_BUILD
			lli_malloc_addr = (u64 *)((u64)malloc(LLI_SIZE * lli_num));
#else
			lli_malloc_addr = (u64 *)CVIMMAP_ION_ADDR;
			memset(lli_malloc_addr, 0x0, LLI_SIZE * lli_num);
#endif
			if (!lli_malloc_addr)
				printf("alloacte lli_malloc_addr failed, alloc size: %d\n", LLI_SIZE * lli_num);

			llp_addr = (u64 *)(((uintptr_t)lli_malloc_addr + 64) & (~0x3f));
			tx_lli_start_addr = llp_addr;
		} else {
			if (first_lli)
				llp_addr = tx_lli_start_addr;
			else
				llp_addr += (LLI_SIZE / 8);
		}

		LLI_WRITE(llp_addr, 0x0, src_addr + (offset / 4)); //sar, offset is 64 bytes
		LLI_WRITE(llp_addr, 0x8, reg); //dar
		LLI_WRITE(llp_addr, 0x20, ctl | 1UL << 63); //ctl, set lli valid bit
		LLI_WRITE(llp_addr, 0x10, xfer_count - 1); //block ts - 1

		if (!first_lli)
			LLI_WRITE(dma_addr_prev, 0x18, (u64)llp_addr | 1);//set llp mem addr and select m1

		dma_addr_prev = llp_addr;
		first_lli = 0;
		i++;
	}
	LLI_WRITE(dma_addr_prev, 0x20, ctl | 3LL << 62);//set llp the last block

	flush_dcache_all();
	__asm__ __volatile__("" : : : "memory");
	debug("TX allocate %d LLI\n", i);
}

/*
2.3 do start the dma transfer
	start initialize
	channel cfg setting
	dma on
*/
int dma_start_transfer(unsigned int p_dev)
{
	u64 cfg;
	unsigned int channel;
	u64 chan_en;
	u64 val = 0;
	int ret = 0;

	DMAC_WRITE(0x10, 0x3); //enable dma

	cfg = 15UL << 55
		| 15UL << 59 //src and dst request limit
		| 7UL << 49 // highest priority,0 is lowest priority
		| 0x3 << 2 //Linked List type of destination
		| 0x3 << 0 //0b11 to enable link list mode of source
		| 1UL << 32 // mem2dev dmac is the flow control
		| 0UL << 36 //Destination Software or Hardware Handshaking select
		| 0UL << 35; //Source Software or Hardware Handshaking Select

	// do start dma,

	switch (p_dev) {
	case DMA_I2S0:
		channel =  DMA_CH1;
		cfg |= 1UL << 39 // i2s0 tx src hw handshake interface
			| 1UL << 44; // i2s0 tx dst hw handshake interface
		break;
	case DMA_I2S1:
		channel = DMA_CH3;
		cfg |= 3UL << 39 // i2s1 tx src hw handshake interface
			| 3UL << 44; // i2s1 tx dst hw handshake interface
		break;
	case DMA_I2S2:
		cfg |= 5UL << 39 // i2s2 tx src hw handshake interface
			| 5UL << 44; // i2s2 tx dst hw handshake interface
		break;
	case DMA_I2S3:
		cfg |= 7UL << 39 // i2s3 tx src hw handshake interface
			| 7UL << 44; // i2s3 tx dst hw handshake interface
		break;
	case DMA_I2S4:
	case DMA_I2S5:
	case DMA_DW_I2S:
		channel =  DMA_CH1;
		cfg |= 1UL << 39 // i2s0 tx src hw handshake interface
			| 1UL << 44; // i2s0 tx dst hw handshake interface
		break;
	case DMA_I2C0:
		channel = DMA_CH5;
		cfg |= 5UL << 39 // i2c0 tx src hw handshake interface
			| 5UL << 44; // i2c0 tx dst hw handshake interface
		break;
	case DMA_I2C1:
	case DMA_I2C2:
	case DMA_I2C3:
	case DMA_I2C4:
		break;
	case DMA_SPI0:
		channel = DMA_CH3;
		cfg |= 3UL << 39 // spi0 tx src hw handshake interface
			| 3UL << 44; // spi0 tx dst hw handshake interface
		break;
	default:
		ret = -2;
		printf("unsupport dma dev %d\n", p_dev);
		return ret;
	}

	chan_en = DMAC_READ(0x18); // CH_EN
//	for (channel =0;channel < 7;channel++) {
	if (!((chan_en >> channel) & 0x1)) { //check if channel is inactive
		DMA_CHAN_WRITE(channel, 0x120, cfg);//cfg
		DMA_CHAN_WRITE(channel, 0x128, tx_lli_start_addr);//set the first llp mem addr to CHANNEL LLP reg

		val = (1 << channel) << 8 | (1 << channel);
		DMAC_WRITE(0x18, val); //chan enable bit and chan enable enable bit
		u32 cnt = 0;

		chan_en = 1 << channel;

		do {
			udelay(1);
			if (cnt++ > 10000000) { //set timeout to 10s
				printf("M2P DMA timeout\n");
				ret = -1;
				break; //timeout
			}
		} while (chan_en & DMAC_READ(0x18));

		// printf("%s, AFTER. chan_en=0x%llx,s\n",__func__, DMAC_READ(0x18));
	} else {
		printf("DMA channel %d is busy\n", channel);
	}

/*3, stop dma */
	DMAC_WRITE(0x10, 0);
	if (!lli_malloc_addr) {
		free(lli_malloc_addr);
		lli_malloc_addr = NULL;
		tx_lli_start_addr = NULL;
		rx_lli_start_addr = NULL;
	}
	return ret;
}

void dma_dev2mem_setting(unsigned int *src_addr, unsigned int len, unsigned int *reg, unsigned int p_dev)
{
	u64 ctl;
	unsigned int first_lli = 1;
	u64 *llp_addr;
	size_t offset;
	size_t xfer_count;
	unsigned int trans_width = 0x2;
	unsigned int i = 0;

/*
2.1 prepare work, default ctl setting
	prepare ctl reg
	source and dest data width, tr_width
	data width = 0x2 AXI fix read or write one DWORD 32bits
	reg_width 0x0:bits,0x1:16bits
	src inc and dst no change when dma do mem2dev copy
*/

	switch (p_dev) {
	case DMA_I2S0:
	case DMA_I2S1:
	case DMA_I2S2:
	case DMA_I2S3:
	case DMA_I2S4:
	case DMA_I2S5:
	case DMA_DW_I2S:
		ctl = 0x3 << 14 //SRC MSIZE
			| 0x3 << 18 //DST MSIZE
			| 0x2 << 8  //src_tr_width
			| 0x2 << 11 //dst_tr_width
			| 0xFUL << 48
			| 0x1UL << 47
			| 0x0UL << 39
			| 0x1UL << 38;
		break;
	case DMA_I2C0:
	case DMA_I2C1:
	case DMA_I2C2:
	case DMA_I2C3:
	case DMA_I2C4:
		ctl = 0x3 << 14 //SRC MSIZE
			| 0x3 << 18 //DST MSIZE
			| 0x2 << 8  //src_tr_width
			| 0x2 << 11; //dst_tr_width
		break;
	case DMA_SPI0:
		ctl = 0x3 << 14 //SRC MSIZE
			| 0x3 << 18 //DST MSIZE
			| 0x2 << 8  //src_tr_width
			| 0x2 << 11; //dst_tr_width
		break;
	case DMA_AUDSRC:
		ctl = 0x1 << 14 //SRC MSIZE
			| 0x1 << 18 //DST MSIZE
			| 0x2 << 8  //src_tr_width
			| 0x2 << 11 //dst_tr_width
			| 0xFUL << 48
			| 0x1UL << 47
			| 0x0UL << 39
			| 0x1UL << 38;
		break;
	default:
		return;
	}

//	DMAC_WRITE(0x10, 0x0);	//turn off dma first

	ctl |= 0x1 << 0 //SMS
		| 0x1 << 2 //DMS
		| 0X1 << 4 //SRC no change
		| 0X0 << 6; //DST inc

/* 2.2 prepare link list setting
	len: to send data size
	xfer_count: the count of link list items
	trans_width: data_width,32bits fixed
	src_addr, dst_addr: memory addr
	trans_width = 0x2
*/
	int lli_num = (len + 64 - 1) / 64 + 1;

	for (offset = 0; offset < len; offset = (offset + (xfer_count << trans_width))) {
		if (xfer_count == 0)
			xfer_count = 1;
		xfer_count = min_t(size_t, (len - offset) >> trans_width, 16);
		if (!rx_lli_start_addr) {
#ifndef CONFIG_SPL_BUILD
			lli_malloc_addr = (u64 *)((u64)malloc(LLI_SIZE * lli_num));
#else
			lli_malloc_addr = (u64 *)CVIMMAP_ION_ADDR;
			memset(lli_malloc_addr, 0x0, LLI_SIZE * lli_num);
#endif
			if (!lli_malloc_addr)
				printf("alloacte lli_malloc_addr failed, alloc size: %d\n", LLI_SIZE * lli_num);

			llp_addr = (u64 *)(((uintptr_t)lli_malloc_addr + 64) & (~0x3f));
			rx_lli_start_addr = llp_addr;
		} else {
			if (first_lli)
				llp_addr = rx_lli_start_addr;
			else {
				llp_addr += (LLI_SIZE / 8);
			}
		}
		LLI_WRITE(llp_addr, 0x0, reg); //sar
		LLI_WRITE(llp_addr, 0x8, src_addr + (offset / 4)); //dar, offset is 64 bytes

		LLI_WRITE(llp_addr, 0x20, ctl | 1UL << 63); //ctl, set lli valid bit

		LLI_WRITE(llp_addr, 0x10, xfer_count - 1); //block ts - 1

		if (!first_lli)
			LLI_WRITE(dma_addr_prev, 0x18, (u64)llp_addr | 1);//set llp mem addr and select m1

		dma_addr_prev = llp_addr;
		first_lli = 0;
		i++;
	}
	LLI_WRITE(dma_addr_prev, 0x20, ctl | 3LL << 62);//set llp the last block
	flush_dcache_all();
	__asm__ __volatile__("" : : : "memory");
	debug("RX allocate %d LLI\n", i);
}

int dma_start_receive(unsigned int p_dev)
{
	u64 cfg;
	unsigned int channel;
	u64 chan_en;
	u64 val = 0;
	int ret = 0;

	DMAC_WRITE(0x10, 0x3); //enable dma

	cfg = 15UL << 55
		| 15UL << 59 //src and dst request limit
		| 7UL << 49 // highest priority,0 is lowest priority
		| 0x3 << 2 //Linked List type of destination
		| 0x3 << 0 //0b11 to enable link list mode of source
		| 2UL << 32 // dev2mem dmac is the flow control,i2s has't fc
		| 0UL << 36 //Destination Software or Hardware Handshaking select
		| 0UL << 35; //Source Software or Hardware Handshaking Select

	switch (p_dev) {
	case DMA_I2S0:
		channel =  DMA_CH0;
		cfg |= 0UL << 39 // i2s0 rx src hw handshake interface
			| 0UL << 44; // i2s0 rx dst hw handshake interface
		break;
	case DMA_I2S1:
		channel =  DMA_CH2;
		cfg |= 2UL << 39 // i2s1 rx src hw handshake interface
			| 2UL << 44; // i2s1 rx dst hw handshake interface
		break;
	case DMA_I2S2:
		channel =  DMA_CH4;
		cfg |= 4UL << 39 // i2s2 rx src hw handshake interface
			| 4UL << 44; // i2s2 rx dst hw handshake interface
		break;
	case DMA_I2S3:
		cfg |= 6UL << 39 // i2s3 rx src hw handshake interface
			| 6UL << 44; // i2s3 rx dst hw handshake interface
		break;
	case DMA_I2S4:
	case DMA_I2S5:
	case DMA_DW_I2S:
		channel =  DMA_CH0;
		cfg |= 0UL << 39 // i2s0 rx src hw handshake interface
			| 0UL << 44; // i2s0 rx dst hw handshake interface
		break;
	case DMA_I2C0:
		channel =  DMA_CH4;
		cfg |= 4UL << 39 // i2c0 rx src hw handshake interface
			| 4UL << 44; // i2c0 rx dst hw handshake interface
		break;
	case DMA_I2C1:
	case DMA_I2C2:
	case DMA_I2C3:
	case DMA_I2C4:
		break;
	case DMA_SPI0:
		channel =  DMA_CH2;
		cfg |= 2UL << 39 // spi0 rx src hw handshake interface
			| 2UL << 44; // spi0 rx dst hw handshake interface
		break;
	default:
		ret = -2;
		printf("unsupport dma dev %d\n", p_dev);
		return ret;
	}
	// do start dma

	chan_en = DMAC_READ(0x18); // CH_EN

	if (!((chan_en >> channel) & 0x1)) { //check if channel is inactive
		DMA_CHAN_WRITE(channel, 0x120, cfg);//cfg
		DMA_CHAN_WRITE(channel, 0x128, rx_lli_start_addr);//set the first llp mem addr to CHANNEL LLP reg
		val = (1 << channel) << 8 | (1 << channel);
		DMAC_WRITE(0x18, val); //chan enable bit and chan enable enable bit
		chan_en = 1 << channel;

		// wait dma trans complete
		u32 cnt = 0;

		do {
			udelay(1);
			val = DMAC_READ(0x18);
			//	debug("%s, AFTER. chan_en=0x%lx\n",__func__, val);

			if (cnt++ > 10000000) { //set timeout to 10s
				printf("P2M DMA timeout\n");
				ret = -1;
				break; //timeout
			}

			//printf("[r]D0:0x%x\n", readl(0x0291200d0));
		} while (chan_en & val);
	} else
		printf("DMA channel %d is busy\n", channel);

/* 3, stop dma */
	// dma turn off
	DMAC_WRITE(0x10, 0);
	if (!lli_malloc_addr) {
		free(lli_malloc_addr);
		lli_malloc_addr = NULL;
		tx_lli_start_addr = NULL;
		rx_lli_start_addr = NULL;
	}
	return ret;
}
