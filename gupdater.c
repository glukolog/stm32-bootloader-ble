#include <string.h>
#include "gupdater.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_flash_ex.h"
//#include "flash.h"

extern UART_HandleTypeDef huart1;

/*
 * every packet received 20 bytes
 * 00 crc = xor next 19 bytes with 0x00
 * 01 address byte 2
 * 02 address byte 1
 * 03 address byte 0
 * 04 .. (RX_PKT_SZ - 4) bytes of binary
 *
 * if byte 00 == 0xFF means command
 * crc ff 01 xx uint32_t filesize uint32_t file_crc + 8 byte of zero
 * crc ff 02 xx
 */

uint8_t flash_block_erase(uint32_t address) {
  uint8_t status = 0;

  if (address > FLASH_BANK1_END) return 2;
/*
  FLASH->CR |= FLASH_CR_PER;
  //while (FLASH->SR & FLASH_SR_BSY);
  FLASH->AR = address;
  FLASH->CR |= FLASH_CR_STRT;
  while (FLASH->SR & FLASH_SR_BSY);
  FLASH->CR &= ~FLASH_CR_PER;

  if ((FLASH->SR & FLASH_SR_EOP) == 0) status = 1;
*/

  FLASH_EraseInitTypeDef erase_init;
  uint32_t error = 0u;
  erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
  erase_init.PageAddress = address;
  erase_init.Banks = FLASH_BANK_1;
  erase_init.NbPages = 1;
  if (HAL_FLASHEx_Erase(&erase_init, &error) != HAL_OK) status = 1;

  return status;
}


uint8_t checkCRC(uint8_t *buf, int len) {
	uint8_t crc = 0;
	for(int i = 0 ; i < len ; i++) crc ^= buf[i];
	return crc;
}

uint8_t updater_receiver(void) {
	uint8_t buf_rx[RX_PKT_SZ] = {0}, buf_tx[RX_PKT_SZ] = {0};
	//uint8_t flash_buf[1024], flash_pos = 0;
	uint32_t fwsize = 0, fwpos = 10, fw_crc = 0, my_crc = 0;
	uint8_t status = 1, wait_first = 1, retry = 0;

	//HAL_FLASH_Unlock();
	//FLASH_PageErase(0x08002000);
	//flash_block_erase(0x08002000);

	while (status) {
		if (HAL_UART_Receive(&huart1, buf_rx, RX_PKT_SZ, UART_TIMEOUTX) == HAL_TIMEOUT) {
			buf_tx[0] = CH_WAIT;
			if (!wait_first) {
				buf_tx[0] = CH_RETRY;
				if (retry > MAX_RETRY) { buf_tx[0] = CH_ERR; status = 0; } else retry++;
			}
			HAL_UART_Transmit(&huart1, buf_tx, 1u, UART_TIMEOUTX);
			continue;
		}
		if (checkCRC(buf_rx + 1, RX_PKT_SZ - 1) != buf_rx[0]) {
			buf_tx[0] = CH_BADCRC;
			HAL_UART_Transmit(&huart1, buf_tx, 1u, UART_TIMEOUTX);
			continue;
		}
		retry = 0;
		if (buf_rx[1] == 0xFF) {
			uint32_t *ptr32 = (uint32_t *)(buf_rx);
			switch(buf_rx[2]) {
			case 1 :
				wait_first = 0;
				fwsize = ptr32[1];
				fw_crc = ptr32[2];
				buf_tx[0] = CH_OK;
				/*
				//if (flash_erase(FLASH_APP_START_ADDRESS) != FLASH_OK) {
				if (!flash_block_erase(0)) {
					buf_tx[0] = CH_ERRF;
					status = 0;
				} //else HAL_FLASH_Unlock();
				*/
				HAL_UART_Transmit(&huart1, buf_tx, 1u, UART_TIMEOUTX);
				break;
			}
		} else {
			uint32_t addr = (buf_rx[1] << 16) + (buf_rx[2] << 8) + buf_rx[3];
			uint32_t *ptr32 = (uint32_t *)(buf_rx + 4);
			buf_tx[0] = CH_OK;
			if (addr != fwpos) {
				if ((addr & (FLASH_PAGE_SIZE - 1)) == 0) {
					HAL_FLASH_Unlock();
					if (flash_block_erase(addr + FLASH_APP_START_ADDRESS) != 0) {
						buf_tx[0] = CH_ERRF;
						status = 0;
					} //else HAL_FLASH_Unlock();
					HAL_Delay(50);
					HAL_FLASH_Lock();
				}
				for (uint32_t i = 0; i < ((RX_PKT_SZ / 4) - 1); i++){
					uint32_t faddr = FLASH_APP_START_ADDRESS + addr + (i * 4);
					my_crc ^= ptr32[i];
					HAL_FLASH_Unlock();
					if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, faddr, ptr32[i]) != HAL_OK) {
						buf_tx[0] = CH_ERRF;
						status = 0;
					}
					HAL_FLASH_Lock();
					#ifdef FLASH_VERIFY
					if (ptr32[i] != (*(volatile uint32_t*)faddr)) {
						buf_tx[0] = CH_ERRF;
						status = 0;
					}
					#endif
				}
				//if (addr && fwpos + (RX_PKT_SZ - 4) != addr) { retry++; buf_tx[0] = CH_RETRY; } else retry = 0;
				//if (retry > MAX_RETRY) { buf_tx[0] = CH_ERR; status = 0; }
				fwpos = addr;
				if (addr + (RX_PKT_SZ - 4) >= fwsize) {
					buf_tx[0] = (fw_crc != my_crc) ? CH_ERR : CH_END;
					status = 0;
				}
			}
			HAL_UART_Transmit(&huart1, buf_tx, 1u, UART_TIMEOUTX);
		}
	}
	HAL_FLASH_Lock();
	return buf_tx[0];
}

void flash_jump_to_app(void) {
	//typedef void (*fnc_ptr)(void);
  /* Function pointer to the address of the user application. */
  fnc_ptr jump_to_app;
  //void *jump_to_app(void);
  jump_to_app = (fnc_ptr)(*(volatile uint32_t*) (FLASH_APP_START_ADDRESS+4u));
  HAL_DeInit();
  /* Change the main stack pointer. */
  __set_MSP(*(volatile uint32_t*)FLASH_APP_START_ADDRESS);
  jump_to_app();
}
