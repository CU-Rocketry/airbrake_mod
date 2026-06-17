/*
 * flash.c
 *
 *  Created on: Apr 21, 2026
 *      Author: Sigmond
 */

#include "flash.h"
#include <string.h>
#include "state.h"
#include "packets.h"
#include <stdio.h>
#include "telemetry.h"

static flash_t *flash;

// Resets the flash and enable QSPI. Must be run after MX_OSPI_Init
void flash_init(flash_t *handle) {
	flash = handle;

	/* After MX_OSPI_Init(): */

	// 1. Reset the device to a known state (recover from soft resets)
	W25Q_EnableReset(flash->hospi);
	W25Q_ResetDevice(flash->hospi);

	// 2. Enable Quad bit (QE) in Status Register 2 for Quad I/O operations
	// Using Volatile Write Enable to avoid wearing out the flash on every boot
	W25Q_VolatileSrWriteEnable(flash->hospi);
	W25Q_WriteStatusRegister(flash->hospi, W25Q_SR2, W25Q_SR2_QE);
}

//void flash_write_page(flash_t *flash, uint8_t *data)
//{
//  W25Q_WriteEnable(flash->hospi);
//  // Start DMA transfer, the rest is handled in callbacks
//  W25Q_PageProgramQuadInput_DMA(flash->hospi, flash->address, W25Q_PAGE_SIZE, flash->page_buf);
//  flash->address += W25Q_PAGE_SIZE; // increment address for next write
//  flash->page_buf_idx = 0; // reset buffer index to 0 for next page
//}



// 1. Called when DMA transfer (Write) is complete
void HAL_OSPI_TxCpltCallback(OSPI_HandleTypeDef *hospi)
{
    if (hospi == flash->hospi) { // if caused by this peripheral
        flash->state = FLASH_STATE_BUSY_WRITE; // DMA SPI transfer done but flash controller still writing internally
        W25Q_BusyFlagPolling_IT(hospi); // poll with interrupts the status register until ready
    }
}

// 2. Called when Flash BUSY flag clears (Write finished)
void HAL_OSPI_StatusMatchCallback(OSPI_HandleTypeDef *hospi)
{
    if (hospi == flash->hospi) { // if caused by this peripheral
        flash->state = FLASH_STATE_READY; // write operation done
    }
}

// 3. Called when Read is complete
//void HAL_OSPI_RxCpltCallback(OSPI_HandleTypeDef *hospi)
//{
//  // If using H7/F7: SCB_InvalidateDCache_by_Addr(rxData, W25Q_PAGE_SIZE);
//  // Set a flag and process in the main loop
//  isDataReady = 1;
//}

uint8_t flash_check_erased() {
    uint8_t buffer[W25Q_PAGE_SIZE]; // Reduced to 256 bytes to prevent stack overflow
    uint32_t address = 0;

    while (address < W25Q128JV_SIZE) {
        flash_read_blocking(address, buffer, sizeof(buffer)); // read one page

        for (uint16_t i = 0; i < sizeof(buffer); i++) { // check all are 0xFF
            if (buffer[i] != 0xFF) {
                return 0; // immediately return false if non erased byte found
            }
        }
        address += sizeof(buffer); // move to next page
    }

    return 1; // if we make it, entire chip is erased
}

uint8_t flash_should_add() {
	if (flash->address >= W25Q128JV_SIZE) // is this right?
		return 0;

    // Logging rate based on state
    flash->prescaler_max = 100; // default to 1 Hz on the pad
    if (STATE_FLAG_GET(FLAG_LAUNCHED))
		flash->prescaler_max = 1; // 100 Hz in flight
	if (STATE_FLAG_GET(FLAG_APOGEE))
		flash->prescaler_max = 100; // 1 Hz after apogee

    flash->prescaler_cnt++;
    if (flash->prescaler_cnt >= flash->prescaler_max) {
        flash->prescaler_cnt = 0;
        return 1;
    }

    return 0;
}

void flash_pkt_buf_add() {
    if (flash->address >= W25Q128JV_SIZE) return;

    uint8_t next_idx = (flash->pkt_buf_write_idx + 1) % FLASH_PKT_BUF_SIZE;
    if (next_idx != flash->pkt_buf_read_idx) { // buffer is not full
        flash_packet_build(&(flash->pkt_buf[flash->pkt_buf_write_idx]));
        flash->pkt_buf_write_idx = next_idx;
    } else {
        telemetry_log(LOG_LVL_ERROR, "Flash write buffer overflow");
    }
}

void flash_packet_build(flash_packet_t *dest) {
	dest->t = global_state.t;
	dest->elapsed_t = global_state.elapsed_t;

	dest->flags = global_state.flags;

	dest->batt_v = global_state.batt_v;
	dest->batt_i = global_state.batt_i;

	memcpy(dest->accel_ms2, global_state.accel_ms2, sizeof(dest->accel_ms2));
	memcpy(dest->omega_rads, global_state.omega_rads, sizeof(dest->omega_rads));
	memcpy(dest->mag_mgauss, global_state.mag_mgauss, sizeof(dest->mag_mgauss));
	dest->pres_pa = global_state.pres_pa;

	memcpy(dest->accel_b, global_state.accel_b, sizeof(dest->accel_b));
	memcpy(dest->omega_b, global_state.omega_b, sizeof(dest->omega_b));
	memcpy(dest->mag_b, global_state.mag_b, sizeof(dest->mag_b));

	memcpy(dest->quat, global_state.quat, sizeof(dest->quat));
	memcpy(dest->accel_e, global_state.accel_e, sizeof(dest->accel_e));

	dest->p_ground = global_state.p_ground;
	dest->alt_agl = global_state.alt_agl;
	dest->vel_z = global_state.vel_z;

	dest->predicted = global_state.predicted;
	dest->output = global_state.output;
	dest->p_contrib = global_state.p_contrib;
	dest->i_contrib = global_state.i_contrib;

	dest->servo_cmd = global_state.servo_cmd;
	dest->servo_fdbk = global_state.servo_fdbk;
}

// Processes the queue and initiates DMA transfers
// Call this repeatedly in the main loop
void flash_process() {
    // return if flash not ready, or there is no new data to write, or the flash is full already
	if (flash->state != FLASH_STATE_READY || flash->pkt_buf_read_idx == flash->pkt_buf_write_idx || flash->full) {
		return;
	}

	// check for flash full
    if (flash->address >= W25Q128JV_SIZE) {
		flash->full = 1;
		telemetry_log(LOG_LVL_WARNING, "Flash memory full\r\n");
		return;
    }

    flash_packet_t *packet = &(flash->pkt_buf[flash->pkt_buf_read_idx]); // get pointer to packet to write
	flash->pkt_buf_read_idx = (flash->pkt_buf_read_idx + 1) % FLASH_PKT_BUF_SIZE; // pop packet by incrementing read pointer past it

	memset(flash->dma_page_buf, 0, W25Q_PAGE_SIZE); // zero the buffer
	memcpy(flash->dma_page_buf, packet, sizeof(flash_packet_t)); // write the packet into the beginning of the buffer

    SCB_CleanDCache_by_Addr((uint32_t*)flash->dma_page_buf, W25Q_PAGE_SIZE); // clean D-cache before DMA reads it

    flash->state = FLASH_STATE_BUSY_DMA; // advance state machine

    W25Q_WriteEnable(flash->hospi);
    // Start DMA transfer, the rest is handled in callbacks
    W25Q_PageProgramQuadInput_DMA(flash->hospi, flash->address, W25Q_PAGE_SIZE, flash->dma_page_buf);

    flash->address += W25Q_PAGE_SIZE; // next page
}

// Read data
void flash_read_blocking(uint32_t addr, uint8_t *out, uint32_t size) {
    while (W25Q_GetState(flash->hospi) != W25Q_STATE_READY); // Wait for any background writes

    W25Q_ReadData_DMA(flash->hospi, addr, size, out);

    // Wait for the DMA read to finish
    while (W25Q_GetState(flash->hospi) != W25Q_STATE_READY); // wait for DMA reading to be done

    SCB_InvalidateDCache_by_Addr((uint32_t*)out, size); // invalidate cache so CPU reads from memory not cache
}

// Write data
void flash_write_blocking(uint32_t addr, uint8_t *data, uint16_t size) {
    while (W25Q_GetState(flash->hospi) != W25Q_STATE_READY);

    memset(flash->dma_page_buf, 0xFF, W25Q_PAGE_SIZE);
    memcpy(flash->dma_page_buf, data, size);
    SCB_CleanDCache_by_Addr((uint32_t*)flash->dma_page_buf, W25Q_PAGE_SIZE);

    W25Q_WriteEnable(flash->hospi);
    W25Q_PageProgramQuadInput_DMA(flash->hospi, addr, size, flash->dma_page_buf);

    while (W25Q_GetState(flash->hospi) != W25Q_STATE_READY);
}

// Erase 4kb sector starting at addr
void flash_erase_sector(uint32_t addr) {
    while (W25Q_GetState(flash->hospi) != W25Q_STATE_READY);
    W25Q_WriteEnable(flash->hospi);
    W25Q_Erase4KB(flash->hospi, addr);
    while (W25Q_GetState(flash->hospi) != W25Q_STATE_READY);
}

// Erases entire flash chip, takes a while!!
void flash_erase_chip() {
    while (W25Q_GetState(flash->hospi) != W25Q_STATE_READY);
    W25Q_WriteEnable(flash->hospi);
    W25Q_ChipErase(flash->hospi);
    while (W25Q_GetState(flash->hospi) != W25Q_STATE_READY);

    flash->address = 0;
    flash->full = 0;
    flash->pkt_buf_read_idx = 0;
    flash->pkt_buf_write_idx = 0;
}
