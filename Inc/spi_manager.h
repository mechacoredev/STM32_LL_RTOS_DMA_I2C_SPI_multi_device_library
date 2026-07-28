/*
 * spi_manager.h
 *
 *  Created on: Jul 21, 2026
 *      Author: Enes
 */

#ifndef INC_SPI_MANAGER_H_
#define INC_SPI_MANAGER_H_

#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_gpio.h"
#include "cmsis_os.h"
#include "stdbool.h"
#include "stdint.h"

typedef enum{
	_spi_manager_ok = 0,
	_spi_manager_fail,
	_spi_manager_busy,
	_spi_manager_timeout,
	_spi_manager_size_0,
	_spi_manager_uninited_struct,
}spi_manager_return_status;

#define spi_manager_timeout_ms 100

// Bir SPI hattını (mutex + semaphore ile) kütüphaneye tanıtma
void spi_manager_assign_bus(SPI_TypeDef* spi_handle, osMutexId_t mutex, osSemaphoreId_t sem);

// Ham (protokolden bağımsız) full-duplex transfer fonksiyonları
// txdata ve rxdata aynı 'size' uzunluğunda olmalı. txdata'yı sadece okuma
// yapıyorsan dummy byte (0xFF) ile doldurman senin (rc522.c) sorumluluğunda.
spi_manager_return_status spi_manager_transfer_poll(SPI_TypeDef* spi_handle, GPIO_TypeDef* cs_port, uint32_t cs_pin, uint8_t* txdata, uint8_t* rxdata, uint16_t size);

spi_manager_return_status spi_manager_transfer_dma(SPI_TypeDef* spi_handle, DMA_TypeDef* dma_handle, uint32_t rx_stream, uint32_t tx_stream, GPIO_TypeDef* cs_port, uint32_t cs_pin, uint8_t* txdata, uint8_t* rxdata, uint16_t size);

bool spi_manager_is_dma_busy(SPI_TypeDef* spi_handle);
void spi_manager_dma_handler(DMA_TypeDef* dma_handle, uint32_t dma_stream);
void spi_manager_error_handler(SPI_TypeDef* spi_handle);

#endif /* INC_SPI_MANAGER_H_ */
