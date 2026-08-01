#ifndef INC_NRF24L01_H_
#define INC_NRF24L01_H_

#include "spi_manager.h"
#include "cmsis_os.h"
#include "stdint.h"
#include "stddef.h"

typedef enum{
	_nrf24l01_ok = 0,
	_nrf24l01_fail,
	_nrf24l01_timeout,
	_nrf24l01_device_not_found
}nrf24l01_return_status;

// Kullanıcı Konfigürasyonu (bme280_user_configs gibi)
typedef struct{
	SPI_TypeDef* spi_handle;
	DMA_TypeDef* dma_handle;
	uint32_t rx_stream;
	uint32_t tx_stream;

	GPIO_TypeDef* ce_port;
	uint32_t ce_pin;
	GPIO_TypeDef* csn_port;
	uint32_t csn_pin;

    // DMA'nın bitişini bekleyeceğimiz zil (spi_manager'a verdiğimiz ile aynı olmalı)
	osSemaphoreId_t spi_semaphore;
	// EXTI kesmesini bekleyeceğimiz Görev (Task)
	osThreadId_t notify_task;
	uint32_t notify_flag;     // YENİ: MPU6050'deki gibi hangi bayrağı kaldıracağımızı belirler
}nrf24l01_user_configs;

struct nrf24l01_t;
typedef struct nrf24l01_t* nrf24l01_handle_t;

// Kullanılacak Fonksiyon Prototipleri
nrf24l01_handle_t nrf24l01_init(nrf24l01_user_configs* config);

// DMA Tabanlı Asenkron Veri Transferleri
nrf24l01_return_status nrf24l01_send_dma(nrf24l01_handle_t dev, uint8_t *data, uint8_t length);
nrf24l01_return_status nrf24l01_receive_dma(nrf24l01_handle_t dev, uint8_t *data, uint8_t length);

// Durum Kontrolleri
void nrf24l01_clear_interrupts(nrf24l01_handle_t dev);
void nrf24l01_start_listening(nrf24l01_handle_t dev);
void nrf24l01_stop_listening(nrf24l01_handle_t dev);

void nrf24l01_assign_interrupt_task(nrf24l01_handle_t dev, osThreadId_t task, uint32_t flag);
void nrf24l01_irq_handler(nrf24l01_handle_t dev);

void flush_rx(nrf24l01_handle_t dev);
void flush_tx(nrf24l01_handle_t dev);

#endif /* INC_NRF24L01_H_ */
