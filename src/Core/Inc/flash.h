/*
 * flash.h
 *
 *  Created on: Apr 21, 2026
 *      Author: Sigmond
 */

#ifndef INC_FLASH_H_
#define INC_FLASH_H_

#include "stm32h7xx_hal.h"

#define FLASH_CMD_WRITE_ENABLE       0x06
#define FLASH_CMD_READ_STATUS_1      0x05
#define FLASH_CMD_READ_JEDEC_ID      0x9F
#define FLASH_CMD_READ_DATA          0x03
#define FLASH_CMD_PAGE_PROGRAM       0x02
#define FLASH_CMD_SECTOR_ERASE_4K    0x20
#define FLASH_CMD_CHIP_ERASE         0xC7

#define FLASH_SR1_BUSY               0x01 // Write In Progress bit

#define W25Q128JV_JEDEC_ID 0xEF4018 // 0xEF for Winbond, 0x40 memory type, 0x18 capacity

typedef struct {
    OSPI_HandleTypeDef *hospi;
} flash_t;

uint32_t flash_read_jedec_id(flash_t *flash);
uint8_t flash_read_status(flash_t *flash);
void flash_wait_for_ready(flash_t *flash);
void flash_write_enable(flash_t *flash);
void flash_erase_sector(flash_t *flash, uint32_t address);
void flash_erase_chip(flash_t *flash);
void flash_write_page(flash_t *flash, uint32_t address, uint8_t *data, uint32_t length);
void flash_read_data(flash_t *flash, uint32_t address, uint8_t *data, uint32_t length);

#endif /* INC_FLASH_H_ */
