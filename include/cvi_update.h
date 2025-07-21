#ifndef __CVI_UPDATE_H__
#define __CVI_UPDATE_H__

#define SECTOR_SIZE 0x200
#define EXTRA_FLAG_SIZE 32
// UART update defines
#define UART_UPDATE_MAGIC 0x4D474E33
#define UART_DL_BAUDRATE 115200

#undef pr_debug
#ifdef DEBUG
#define pr_debug(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
#define pr_debug(fmt, ...)
#endif

int _prgImage(char *file, uint32_t chunk_header_size, char *file_name);

#endif /* __CVI_UPDATE_H__ */
