/*
 * i2c_manager.h
 *
 *  Created on: Jul 20, 2026
 *      Author: Enes
 */

#ifndef INC_I2C_MANAGER_H_
#define INC_I2C_MANAGER_H_

#include "stm32f4xx_ll_i2c.h"
#include "stm32f4xx_ll_dma.h"
#include "cmsis_os.h"
#include "stdbool.h"
#include "stdint.h"
#include "stdlib.h"

typedef struct{
	I2C_TypeDef* i2c_handle;
	DMA_TypeDef* dma_handle;
	uint32_t dma_stream;
	uint8_t dev_addr;
	uint8_t reg_addr;
	uint8_t* rxdata;
	uint16_t size;

	osThreadId_t notify_task;
	uint32_t notify_flag;
}i2c_job_t;

typedef enum{
	_i2c_manager_ok = 0,
	_i2c_manager_fail,
	_i2c_manager_busy,
	_i2c_manager_timeout,
	_i2c_manager_size_0,
	_i2c_manager_uninited_struct,
}i2c_manager_return_status;

#define i2c_manager_timeout_ms 100

// Çoklu Veriyolu Kayıt Fonksiyonu (YENİ)
void i2c_manager_assign_bus(I2C_TypeDef* i2c_handle, osMutexId_t mutex, osSemaphoreId_t sem);

i2c_manager_return_status i2c_manager_read_poll(I2C_TypeDef* i2c_handle, uint8_t dev_addr, uint8_t reg_addr, uint8_t* rxdata, uint16_t size);
i2c_manager_return_status i2c_manager_write_poll(I2C_TypeDef* i2c_handle, uint8_t dev_addr, uint8_t reg_addr, uint8_t* txdata, uint16_t size);
i2c_manager_return_status i2c_manager_check_device(I2C_TypeDef* i2c_handle, uint8_t dev_addr, uint8_t reg_addr, uint8_t device_id);
i2c_manager_return_status i2c_manager_read_dma(DMA_TypeDef* dma_handle, uint32_t dma_stream, I2C_TypeDef* i2c_handle, uint8_t dev_addr, uint8_t reg_addr, uint8_t* rxdata, uint16_t size);

// Artık Hangi Hat Olduğunu Sormamız Gerekiyor (YENİ)
bool i2c_manager_is_dma_busy(I2C_TypeDef* i2c_handle);
void i2c_manager_dma_handler(DMA_TypeDef* dma_handle, uint32_t dma_stream);
void i2c_manager_error_handler(I2C_TypeDef* i2c_handle);

#endif /* INC_I2C_MANAGER_H_ */
