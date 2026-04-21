/*
 * flash.c
 *
 *  Created on: Apr 21, 2026
 *      Author: Sigmond
 */

#include "flash.h"

// Reads JEDEC ID with 1-1-1 standard SPI mode
uint32_t flash_read_jedec_id(flash_t *flash) {
    OSPI_RegularCmdTypeDef sCommand = {0};
    uint8_t id_buf[3] = {0};

    // Configure the command for standard 1-bit SPI (1-1-1)
    sCommand.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
    sCommand.FlashId            = HAL_OSPI_FLASH_ID_1;
    sCommand.Instruction        = FLASH_CMD_READ_JEDEC_ID;
    sCommand.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
    sCommand.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
    sCommand.AddressMode        = HAL_OSPI_ADDRESS_NONE; // Read ID doesn't need an address
    sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode           = HAL_OSPI_DATA_1_LINE;  // Receive data on 1 line
    sCommand.NbData             = 3;                     // We expect 3 bytes back
    sCommand.DummyCycles        = 0;
    sCommand.DQSMode            = HAL_OSPI_DQS_DISABLE;
    sCommand.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

    // Send the instruction
    if (HAL_OSPI_Command(flash->hospi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        return 0; // Command failed
    }

    // Receive the data
    if (HAL_OSPI_Receive(flash->hospi, id_buf, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        return 0; // Receive failed
    }

    // Combine the 3 bytes into a 32-bit integer (Manufacturer ID << 16 | Memory Type << 8 | Capacity)
    return (id_buf[0] << 16) | (id_buf[1] << 8) | id_buf[2];
}

uint8_t flash_read_status(flash_t *flash) {
    OSPI_RegularCmdTypeDef sCommand = {0};
    uint8_t status;

    sCommand.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
    sCommand.FlashId            = HAL_OSPI_FLASH_ID_1;
    sCommand.Instruction        = FLASH_CMD_READ_STATUS_1;
    sCommand.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
    sCommand.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
    sCommand.AddressMode        = HAL_OSPI_ADDRESS_NONE;
    sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode           = HAL_OSPI_DATA_1_LINE;
    sCommand.NbData             = 1;
    sCommand.DummyCycles        = 0;
    sCommand.DQSMode            = HAL_OSPI_DQS_DISABLE;
    sCommand.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

    HAL_OSPI_Command(flash->hospi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
    HAL_OSPI_Receive(flash->hospi, &status, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);

    return status;
}

void flash_wait_for_ready(flash_t *flash) {
    // Poll the BUSY bit until it clears
    while ((flash_read_status(flash) & FLASH_SR1_BUSY) == FLASH_SR1_BUSY) {
        // You could add a timeout or a small delay here if using an RTOS
    }
}

void flash_write_enable(flash_t *flash) {
    OSPI_RegularCmdTypeDef sCommand = {0};

    sCommand.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
    sCommand.FlashId            = HAL_OSPI_FLASH_ID_1;
    sCommand.Instruction        = FLASH_CMD_WRITE_ENABLE;
    sCommand.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
    sCommand.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
    sCommand.AddressMode        = HAL_OSPI_ADDRESS_NONE;
    sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode           = HAL_OSPI_DATA_NONE;
    sCommand.DummyCycles        = 0;
    sCommand.DQSMode            = HAL_OSPI_DQS_DISABLE;
    sCommand.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

    HAL_OSPI_Command(flash->hospi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
}

void flash_erase_sector(flash_t *flash, uint32_t address) {
    OSPI_RegularCmdTypeDef sCommand = {0};

    flash_write_enable(flash);

    sCommand.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
    sCommand.FlashId            = HAL_OSPI_FLASH_ID_1;
    sCommand.Instruction        = FLASH_CMD_SECTOR_ERASE_4K;
    sCommand.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
    sCommand.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
    sCommand.AddressMode        = HAL_OSPI_ADDRESS_1_LINE;
    sCommand.AddressSize        = HAL_OSPI_ADDRESS_24_BITS;
    sCommand.Address            = address;
    sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode           = HAL_OSPI_DATA_NONE;
    sCommand.DummyCycles        = 0;
    sCommand.DQSMode            = HAL_OSPI_DQS_DISABLE;
    sCommand.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

    HAL_OSPI_Command(flash->hospi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);

    // Wait for the erase to complete before returning
    flash_wait_for_ready(flash);
}

void flash_erase_chip(flash_t *flash) {
    OSPI_RegularCmdTypeDef sCommand = {0};

    flash_write_enable(flash);

    sCommand.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
    sCommand.FlashId            = HAL_OSPI_FLASH_ID_1;
    sCommand.Instruction        = FLASH_CMD_CHIP_ERASE;
    sCommand.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
    sCommand.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
    sCommand.AddressMode        = HAL_OSPI_ADDRESS_NONE;
    sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode           = HAL_OSPI_DATA_NONE;
    sCommand.DummyCycles        = 0;
    sCommand.DQSMode            = HAL_OSPI_DQS_DISABLE;
    sCommand.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

    HAL_OSPI_Command(flash->hospi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);

    // WARNING: Chip erase can take up to 40 seconds!
    flash_wait_for_ready(flash);
}

void flash_write_page(flash_t *flash, uint32_t address, uint8_t *data, uint32_t length) {
    OSPI_RegularCmdTypeDef sCommand = {0};

    // Ensure we don't try to write across a page boundary (256 bytes)
    if (length > 256) length = 256;

    flash_write_enable(flash);

    sCommand.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
    sCommand.FlashId            = HAL_OSPI_FLASH_ID_1;
    sCommand.Instruction        = FLASH_CMD_PAGE_PROGRAM;
    sCommand.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
    sCommand.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
    sCommand.AddressMode        = HAL_OSPI_ADDRESS_1_LINE;
    sCommand.AddressSize        = HAL_OSPI_ADDRESS_24_BITS;
    sCommand.Address            = address;
    sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode           = HAL_OSPI_DATA_1_LINE;
    sCommand.NbData             = length;
    sCommand.DummyCycles        = 0;
    sCommand.DQSMode            = HAL_OSPI_DQS_DISABLE;
    sCommand.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

    HAL_OSPI_Command(flash->hospi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
    HAL_OSPI_Transmit(flash->hospi, data, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);

    // Wait for the write to complete
    flash_wait_for_ready(flash);
}

void flash_read_data(flash_t *flash, uint32_t address, uint8_t *data, uint32_t length) {
    OSPI_RegularCmdTypeDef sCommand = {0};

    sCommand.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
    sCommand.FlashId            = HAL_OSPI_FLASH_ID_1;
    sCommand.Instruction        = FLASH_CMD_READ_DATA;
    sCommand.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
    sCommand.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
    sCommand.AddressMode        = HAL_OSPI_ADDRESS_1_LINE;
    sCommand.AddressSize        = HAL_OSPI_ADDRESS_24_BITS;
    sCommand.Address            = address;
    sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode           = HAL_OSPI_DATA_1_LINE;
    sCommand.NbData             = length;
    sCommand.DummyCycles        = 0;
    sCommand.DQSMode            = HAL_OSPI_DQS_DISABLE;
    sCommand.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

    HAL_OSPI_Command(flash->hospi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
    HAL_OSPI_Receive(flash->hospi, data, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
}
