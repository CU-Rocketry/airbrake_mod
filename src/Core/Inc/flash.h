/*
 * flash.h
 *
 *  Created on: Apr 21, 2026
 *      Author: Sigmond
 */

#ifndef INC_FLASH_H_
#define INC_FLASH_H_

#include "state.h"
#include "packets.h"
#include "stm32h7xx_hal.h"
#include "w25q.h"

#define W25Q128JV_JEDEC_ID 0xEF4018 // 0xEF for Winbond, 0x40 memory type, 0x18 capacity
#define W25Q128JV_SIZE (16*1024*1024)
#define FLASH_PKT_BUF_SIZE 16

// state machine as recommended by https://github.com/mpekurin/W25Q_STM32_HAL_Driver/tree/main
typedef enum {
    FLASH_STATE_READY, // ready
    FLASH_STATE_BUSY_DMA, // SPI transfer to flash controller
    FLASH_STATE_BUSY_WRITE // flash write data to sectors
} flash_state_t;

typedef struct {
    OSPI_HandleTypeDef *hospi;

    uint32_t address; // Current page address
    uint8_t full;

    // Flash write rate limiter
    uint8_t prescaler_max; // 1, 10, or 100 for 100 Hz, 10 Hz, or 1 Hz logging, respectively
    uint8_t prescaler_cnt;

	volatile flash_state_t state;
	flash_packet_t pkt_buf[FLASH_PKT_BUF_SIZE];
	uint8_t pkt_buf_write_idx;
	uint8_t pkt_buf_read_idx;
	ALIGN_32BYTES(uint8_t dma_page_buf[W25Q_PAGE_SIZE]); // DMA buffer should be aligned for cortex M7
} flash_t;

void flash_init();

uint8_t flash_should_add();
void flash_pkt_buf_add();
void flash_packet_build(flash_packet_t *dest);
void flash_process();

void flash_read_blocking(uint32_t addr, uint8_t *out, uint32_t size);
void flash_write_blocking(uint32_t addr, uint8_t *data, uint16_t size);
void flash_erase_sector(uint32_t addr);
void flash_erase_chip();

uint8_t flash_check_erased();

#endif /* INC_FLASH_H_ */
