/*
 * Copyright (C) 2018 bitmain
 */

#include "mmio.h"

#define DMA_RX_REQ_SPI_NOR              38
#define DMA_TX_REQ_SPI_NOR              39
#define PRE_FILL_SIZE		64

#define DMAC_BASE0				0x04330000
#define DMAC_BASE				DMAC_BASE0
#define DMA_CHAN_BASE DMAC_BASE

#define DMA_LLI_VALID_MASK		0x8000000000000000
#define DMA_LLI_LAST_MASK		0x4000000000000000
#define DMA_LLI_CTL_OFFSET		0x20
#define DMA_LLI_NEXT_LLP_OFFSET	0x18

#define TOP_DMA0_CH_REMAP0	(TOP_BASE + 0x154)
#define TOP_DMA0_CH_REMAP1	(TOP_BASE + 0x158)
#define TOP_DMA0_CH_REMAP2	(TOP_BASE + 0x15c)
#define TOP_DMA0_CH_REMAP3	(TOP_BASE + 0x160)

#define TOP_DMA1_CH_REMAP0	(TOP_BASE + 0x2b0)
#define TOP_DMA1_CH_REMAP1	(TOP_BASE + 0x2b4)
#define TOP_DMA1_CH_REMAP2	(TOP_BASE + 0x2b8)
#define TOP_DMA1_CH_REMAP3	(TOP_BASE + 0x2bc)

#define TOP_DMA_CH_REMAP0	TOP_DMA0_CH_REMAP0
#define TOP_DMA_CH_REMAP1	TOP_DMA0_CH_REMAP1
#define TOP_DMA_CH_REMAP2	TOP_DMA0_CH_REMAP2
#define TOP_DMA_CH_REMAP3	TOP_DMA0_CH_REMAP3

#define DMA_REMAP_CH0_OFFSET	0
#define DMA_REMAP_CH1_OFFSET	8
#define DMA_REMAP_CH2_OFFSET	16
#define DMA_REMAP_CH3_OFFSET	24
#define DMA_REMAP_CH4_OFFSET	0
#define DMA_REMAP_CH5_OFFSET	8
#define DMA_REMAP_CH6_OFFSET	16
#define DMA_REMAP_CH7_OFFSET	24
#define DMA_REMAP_UPDATE_OFFSET	31
#define DMA_REMAP_CH8_OFFSET	0
#define DMA_REMAP_CH9_OFFSET	8
#define DMA_REMAP_CH10_OFFSET	16
#define DMA_REMAP_CH11_OFFSET	24
#define DMA_REMAP_CH12_OFFSET	0
#define DMA_REMAP_CH13_OFFSET	8
#define DMA_REMAP_CH14_OFFSET	16
#define DMA_REMAP_CH15_OFFSET	24

#define DMA_CH0	0
#define DMA_CH1	1
#define DMA_CH2	2
#define DMA_CH3	3
#define DMA_CH4	4
#define DMA_CH5	5
#define DMA_CH6	6
#define DMA_CH7	7

/* refer to dma_remap.docx to define id to each channel */
#define DMA_RX_REQ_I2S0			0
#define DMA_TX_REQ_I2S0			1
#define DMA_RX_REQ_I2S1			2
#define DMA_TX_REQ_I2S1			3
#define DMA_RX_REQ_I2S2			4
#define DMA_TX_REQ_I2S2			5
#define DMA_RX_REQ_I2S3			6
#define DMA_TX_REQ_I2S3			7
#define DMA_RX_REQ_N_UART0		8
#define DMA_TX_REQ_N_UART0		9
#define DMA_RX_REQ_N_UART1		10
#define DMA_TX_REQ_N_UART1		11
#define DMA_RX_REQ_N_UART2		12
#define DMA_TX_REQ_N_UART2		13
#define DMA_RX_REQ_N_UART3		14
#define DMA_TX_REQ_N_UART3		15
#define DMA_RX_REQ_SPI0			16
#define DMA_TX_REQ_SPI0			17
#define DMA_RX_REQ_SPI1			18
#define DMA_TX_REQ_SPI1			19
#define DMA_RX_REQ_SPI2			20
#define DMA_TX_REQ_SPI2			21
#define DMA_RX_REQ_SPI3			22
#define DMA_TX_REQ_SPI3			23
#define DMA_RX_REQ_I2C0			24
#define DMA_TX_REQ_I2C0			25
#define DMA_RX_REQ_I2C1			26
#define DMA_TX_REQ_I2C1			27
#define DMA_RX_REQ_I2C2			28
#define DMA_TX_REQ_I2C2			29
#define DMA_RX_REQ_I2C3			30
#define DMA_TX_REQ_I2C3			31
#define DMA_RX_REQ_I2C4			32
#define DMA_TX_REQ_I2C4			33
#define DMA_RX_REQ_TDM0			34
#define DMA_TX_REQ_TDM0			35
#define DMA_RX_REQ_TDM1			36
#define DMA_REQ_AUDSRC			37
#define DMA_RX_REQ_SPI_NOR		38
#define DMA_TX_REQ_SPI_NOR		39
#define DMA_RX_REQ_N_UART_4		40
#define DMA_TX_REQ_N_UART_4		41
#define DMA_REQ_SPI_NAND		42

/* define peripheral id */
#define DMA_I2S0		0
#define DMA_I2S1		1
#define DMA_I2S2		2
#define DMA_I2S3		3
#define DMA_I2S4		4
#define DMA_I2S5		5
#define DMA_DW_I2S		6
#define DMA_I2C0		7
#define DMA_I2C1		8
#define DMA_I2C2		9
#define DMA_I2C3		10
#define DMA_I2C4		11
#define DMA_I2C5		12
#define DMA_I2C6		13
#define DMA_I2C7		14
#define DMA_I2C8		15
#define DMA_I2C9		15
#define DMA_SPI0		17
#define DMA_AUDSRC		18

#define LLI_SIZE		0x40

typedef	uintptr_t	size_t;

#define DMAC_WRITE(offset, value) \
	mmio_write_64(DMAC_BASE + offset, (uint64_t)value)

#define DMAC_READ(offset) \
	mmio_read_64(DMAC_BASE + offset)

#define DMA_CHAN_WRITE(chan, offset, value) \
	mmio_write_64((uint64_t)(DMA_CHAN_BASE + ((chan * 0x100) + offset)), (uint64_t)(value))

#define DMA_CHAN_READ(chan, offset) \
	mmio_read_64((uint64_t)(DMA_CHAN_BASE + ((chan * 0x100) + offset)))

#define LLI_WRITE(addr, offset, value) \
	(*(addr + (offset / 8)) = (uint64_t)(value))

#define LLI_READ(addr, offset) \
	mmio_read_64(addr + offset)

void dma_turn_on(void);
void dma_turn_off(void);
void dma_reset(void);

void dma_dev2mem_setting(unsigned int *src_addr, unsigned int len, unsigned int *reg, unsigned int p_dev);

void dma_mem2dev_setting(unsigned int *src_addr, unsigned int len, unsigned int *reg, unsigned int p_dev);

int dma_start_transfer(unsigned int p_dev);

int dma_start_receive(unsigned int p_dev);
