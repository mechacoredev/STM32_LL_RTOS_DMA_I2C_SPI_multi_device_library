/*
 * i2c_manager.c
 *
 *  Created on: Jul 20, 2026
 *      Author: Enes
 */

#include "i2c_manager.h"

typedef enum{
	DMA_STATE_IDLE = 0,
	DMA_STATE_START_SENT,
	DMA_STATE_WRITE_ADDR_SENT ,
	DMA_STATE_REG_ADDR_SENT,
	DMA_STATE_RESTART_SENT,
	DMA_STATE_READ_ADDR_SENT,
	DMA_STATE_READING,
	DMA_STATE_TRANSFER_ERROR,
	DMA_STATE_TRANSFER_COMPLETE,
	DMA_STATE_BUSY,
}dma_statement;

struct dma_management{
	I2C_TypeDef* i2c_handle;
	osMutexId_t mutexHandle;         // YENİ: Her I2C'nin Kendi Kilidi
	osSemaphoreId_t semaphoreHandle; // YENİ: Her I2C'nin Kendi Zili
	DMA_TypeDef* dma_handle;
	uint32_t dma_stream;
	uint8_t dev_addr;
	uint8_t reg_addr;
	uint8_t* rxdata;
	uint16_t size;
	dma_statement dma_state;
};

// YENİ: I2C1, I2C2 ve I2C3 için 3 ayrı yönetim odası (Pool)
static struct dma_management i2c_jobs[3];

// YENİ: Gelen donanımın hangi odada yönetildiğini bulan akıllı fonksiyon
static struct dma_management* get_job(I2C_TypeDef* i2c_handle){
	if(i2c_handle == I2C1) return &i2c_jobs[0];
	if(i2c_handle == I2C2) return &i2c_jobs[1];
	if(i2c_handle == I2C3) return &i2c_jobs[2];
	return NULL;
}

// YENİ: İşletim sistemi ayağa kalkarken kilitleri kütüphaneye kaydetme
void i2c_manager_assign_bus(I2C_TypeDef* i2c_handle, osMutexId_t mutex, osSemaphoreId_t sem){
	struct dma_management* job = get_job(i2c_handle);
	if(job != NULL){
		job->i2c_handle = i2c_handle;
		job->mutexHandle = mutex;
		job->semaphoreHandle = sem;
		job->dma_state = DMA_STATE_IDLE;
	}
}

static uint32_t get_tick(void){
	return osKernelGetTickCount();
}

static uint8_t flag_busy(I2C_TypeDef* i2c_handle){
	return !LL_I2C_IsActiveFlag_BUSY(i2c_handle);
}

static uint8_t flag_sb(I2C_TypeDef* i2c_handle){
	return LL_I2C_IsActiveFlag_SB(i2c_handle);
}

static uint8_t flag_addr(I2C_TypeDef* i2c_handle){
	return LL_I2C_IsActiveFlag_ADDR(i2c_handle);
}

static uint8_t flag_txe(I2C_TypeDef* i2c_handle){
	return LL_I2C_IsActiveFlag_TXE(i2c_handle);
}

static uint8_t flag_rxne(I2C_TypeDef* i2c_handle){
	return LL_I2C_IsActiveFlag_RXNE(i2c_handle);
}

static uint8_t flag_btf(I2C_TypeDef* i2c_handle){
	return LL_I2C_IsActiveFlag_BTF(i2c_handle);
}

static uint8_t flag_addr_or_af(I2C_TypeDef* i2c_handle){
	return (LL_I2C_IsActiveFlag_ADDR(i2c_handle) || LL_I2C_IsActiveFlag_AF(i2c_handle));
}

typedef uint8_t (*i2c_condition_t) (I2C_TypeDef* i2c_handle);

static i2c_manager_return_status wait_for_condition(I2C_TypeDef* i2c_handle, i2c_condition_t condition_met){
	uint32_t start_time = get_tick();
	while(condition_met(i2c_handle) == 0){
		if((get_tick() - start_time) >= i2c_manager_timeout_ms){
			return _i2c_manager_timeout;
		}
		osDelay(1);
	}
	return _i2c_manager_ok;
}

i2c_manager_return_status i2c_manager_read_poll(I2C_TypeDef* i2c_handle, uint8_t dev_addr, uint8_t reg_addr, uint8_t* rxdata, uint16_t size){
	if(size == 0) return _i2c_manager_size_0;

	struct dma_management* job = get_job(i2c_handle);
	if(job == NULL || job->mutexHandle == NULL) return _i2c_manager_uninited_struct;

	if(osMutexAcquire(job->mutexHandle, osWaitForever) != osOK) return _i2c_manager_busy;

	if(wait_for_condition(i2c_handle, flag_busy) != _i2c_manager_ok){
		LL_I2C_GenerateStopCondition(i2c_handle);
		osMutexRelease(job->mutexHandle);
		return _i2c_manager_timeout;
	}

	LL_I2C_GenerateStartCondition(i2c_handle);
	if(wait_for_condition(i2c_handle, flag_sb) != _i2c_manager_ok) goto i2c_error;

	LL_I2C_TransmitData8(i2c_handle, dev_addr & 0xFE);
	if(wait_for_condition(i2c_handle, flag_addr) != _i2c_manager_ok) goto i2c_error;
	LL_I2C_ClearFlag_ADDR(i2c_handle);

	LL_I2C_TransmitData8(i2c_handle, reg_addr);
	if(wait_for_condition(i2c_handle, flag_txe) != _i2c_manager_ok) goto i2c_error;

	LL_I2C_GenerateStartCondition(i2c_handle);
	if(wait_for_condition(i2c_handle, flag_sb) != _i2c_manager_ok) goto i2c_error;

	LL_I2C_TransmitData8(i2c_handle, dev_addr | 1);
	if(wait_for_condition(i2c_handle, flag_addr) != _i2c_manager_ok) goto i2c_error;

	if(size == 1){
		LL_I2C_AcknowledgeNextData(i2c_handle, LL_I2C_NACK);
		LL_I2C_ClearFlag_ADDR(i2c_handle);
		LL_I2C_GenerateStopCondition(i2c_handle);
	}
	else{
		LL_I2C_AcknowledgeNextData(i2c_handle, LL_I2C_ACK);
		LL_I2C_ClearFlag_ADDR(i2c_handle);
		while(size > 1){
			if(wait_for_condition(i2c_handle, flag_rxne) != _i2c_manager_ok) goto i2c_error;
			*rxdata = LL_I2C_ReceiveData8(i2c_handle);
			rxdata++;
			size--;
		}
		LL_I2C_AcknowledgeNextData(i2c_handle, LL_I2C_NACK);
		LL_I2C_GenerateStopCondition(i2c_handle);
	}

	if(wait_for_condition(i2c_handle, flag_rxne) != _i2c_manager_ok) goto i2c_error;
	*rxdata = LL_I2C_ReceiveData8(i2c_handle);

	osMutexRelease(job->mutexHandle);
	return _i2c_manager_ok;

				i2c_error:
				if(LL_I2C_IsActiveFlag_ADDR(i2c_handle)) LL_I2C_ClearFlag_ADDR(i2c_handle);
				LL_I2C_GenerateStopCondition(i2c_handle);
				osMutexRelease(job->mutexHandle);
				return _i2c_manager_timeout;
}

i2c_manager_return_status i2c_manager_write_poll(I2C_TypeDef* i2c_handle, uint8_t dev_addr, uint8_t reg_addr, uint8_t* txdata, uint16_t size){
	if(size == 0) return _i2c_manager_size_0;

	struct dma_management* job = get_job(i2c_handle);
	if(job == NULL || job->mutexHandle == NULL) return _i2c_manager_uninited_struct;

	if(osMutexAcquire(job->mutexHandle, osWaitForever) != osOK) return _i2c_manager_busy;

	if(wait_for_condition(i2c_handle, flag_busy) != _i2c_manager_ok){
		LL_I2C_GenerateStopCondition(i2c_handle);
		osMutexRelease(job->mutexHandle);
		return _i2c_manager_busy;
	}

	LL_I2C_GenerateStartCondition(i2c_handle);
	if(wait_for_condition(i2c_handle, flag_sb) != _i2c_manager_ok) goto i2c_error;

	LL_I2C_TransmitData8(i2c_handle, dev_addr & 0xFE);
	if(wait_for_condition(i2c_handle, flag_addr) != _i2c_manager_ok) goto i2c_error;
	LL_I2C_ClearFlag_ADDR(i2c_handle);

	LL_I2C_TransmitData8(i2c_handle, reg_addr);
	if(wait_for_condition(i2c_handle, flag_txe) != _i2c_manager_ok) goto i2c_error;

	while(size > 1){
		LL_I2C_TransmitData8(i2c_handle, *txdata);
		if(wait_for_condition(i2c_handle, flag_txe) != _i2c_manager_ok) goto i2c_error;
		txdata++;
		size--;
	}

	LL_I2C_TransmitData8(i2c_handle, *txdata);
	if(wait_for_condition(i2c_handle, flag_btf) != _i2c_manager_ok) goto i2c_error;

	LL_I2C_GenerateStopCondition(i2c_handle);
	osMutexRelease(job->mutexHandle);
	return _i2c_manager_ok;

				i2c_error:
				if(LL_I2C_IsActiveFlag_ADDR(i2c_handle)) LL_I2C_ClearFlag_ADDR(i2c_handle);
				LL_I2C_GenerateStopCondition(i2c_handle);
				osMutexRelease(job->mutexHandle);
				return _i2c_manager_timeout;
}

i2c_manager_return_status i2c_manager_check_device(I2C_TypeDef* i2c_handle, uint8_t dev_addr, uint8_t reg_addr, uint8_t device_id){
	struct dma_management* job = get_job(i2c_handle);
	if(job == NULL || job->mutexHandle == NULL) return _i2c_manager_uninited_struct;

	if(osMutexAcquire(job->mutexHandle, osWaitForever) != osOK) return _i2c_manager_busy;

	if(wait_for_condition(i2c_handle, flag_busy) != _i2c_manager_ok){
		LL_I2C_GenerateStopCondition(i2c_handle);
		osMutexRelease(job->mutexHandle);
		return _i2c_manager_timeout;
	}

	LL_I2C_GenerateStartCondition(i2c_handle);
	if(wait_for_condition(i2c_handle, flag_sb) != _i2c_manager_ok) goto i2c_error;

	LL_I2C_TransmitData8(i2c_handle, dev_addr & 0xFE);
	if(wait_for_condition(i2c_handle, flag_addr_or_af) != _i2c_manager_ok) goto i2c_error;

	if(LL_I2C_IsActiveFlag_ADDR(i2c_handle)){
		LL_I2C_ClearFlag_ADDR(i2c_handle);

		LL_I2C_TransmitData8(i2c_handle, reg_addr);
		if(wait_for_condition(i2c_handle, flag_txe) != _i2c_manager_ok) goto i2c_error;

		LL_I2C_GenerateStartCondition(i2c_handle);
		if(wait_for_condition(i2c_handle, flag_sb) != _i2c_manager_ok) goto i2c_error;

		LL_I2C_TransmitData8(i2c_handle, dev_addr | 1);
		if(wait_for_condition(i2c_handle, flag_addr_or_af) != _i2c_manager_ok) goto i2c_error;

		if(LL_I2C_IsActiveFlag_ADDR(i2c_handle)){
			LL_I2C_AcknowledgeNextData(i2c_handle, LL_I2C_NACK);
			LL_I2C_ClearFlag_ADDR(i2c_handle);

			if(wait_for_condition(i2c_handle, flag_rxne) != _i2c_manager_ok) goto i2c_error;
			uint8_t rxdata = LL_I2C_ReceiveData8(i2c_handle);
			LL_I2C_GenerateStopCondition(i2c_handle);
			osMutexRelease(job->mutexHandle);
			if(rxdata == device_id){
				return _i2c_manager_ok;
			}else{
				return _i2c_manager_fail;
			}
		}
		if(LL_I2C_IsActiveFlag_AF(i2c_handle)){
			LL_I2C_ClearFlag_AF(i2c_handle);
			goto i2c_error;
		}
	}
	if(LL_I2C_IsActiveFlag_AF(i2c_handle)){
		LL_I2C_ClearFlag_AF(i2c_handle);
		goto i2c_error;
	}

				i2c_error:
				if(LL_I2C_IsActiveFlag_ADDR(i2c_handle)) LL_I2C_ClearFlag_ADDR(i2c_handle);
				if(LL_I2C_IsActiveFlag_AF(i2c_handle)) LL_I2C_ClearFlag_AF(i2c_handle);
				LL_I2C_GenerateStopCondition(i2c_handle);
				osMutexRelease(job->mutexHandle);
				return _i2c_manager_timeout;
}

bool i2c_manager_is_dma_busy(I2C_TypeDef* i2c_handle){
	struct dma_management* job = get_job(i2c_handle);
	if(job == NULL) return true; // Hata durumunda sistemi meşgul gösterip korur

	if(job->dma_state != DMA_STATE_IDLE) return true;
	return false;
}

static void i2c_dma_cleanup(struct dma_management* job){
	LL_I2C_DisableIT_ERR(job->i2c_handle);
	LL_I2C_DisableDMAReq_RX(job->i2c_handle);
	LL_I2C_DisableLastDMA(job->i2c_handle);

	LL_DMA_DisableIT_TC(job->dma_handle, job->dma_stream);
	LL_DMA_DisableIT_TE(job->dma_handle, job->dma_stream);
	LL_DMA_DisableStream(job->dma_handle, job->dma_stream);
}

i2c_manager_return_status i2c_manager_read_dma(DMA_TypeDef* dma_handle, uint32_t dma_stream, I2C_TypeDef* i2c_handle, uint8_t dev_addr, uint8_t reg_addr, uint8_t* rxdata, uint16_t size){
	if(size == 0) return _i2c_manager_fail;

	struct dma_management* job = get_job(i2c_handle);
	if(job == NULL || job->mutexHandle == NULL) return _i2c_manager_uninited_struct;

	if(osMutexAcquire(job->mutexHandle, osWaitForever) != osOK) return _i2c_manager_busy;

	if(wait_for_condition(i2c_handle, flag_busy) != _i2c_manager_ok){
		LL_I2C_GenerateStopCondition(i2c_handle);
		osMutexRelease(job->mutexHandle);
		return _i2c_manager_busy;
	}

	LL_I2C_DisableIT_BUF(i2c_handle);
	LL_I2C_DisableIT_ERR(i2c_handle);
	LL_I2C_DisableIT_EVT(i2c_handle);
	LL_I2C_DisableDMAReq_RX(i2c_handle);
	LL_I2C_DisableLastDMA(i2c_handle);

	LL_DMA_DisableIT_TC(dma_handle, dma_stream);
	LL_DMA_DisableIT_TE(dma_handle, dma_stream);
	LL_DMA_DisableStream(dma_handle, dma_stream);

	job->dma_handle = dma_handle;
	job->dma_stream = dma_stream;
	job->dev_addr = dev_addr;
	job->reg_addr = reg_addr;
	job->rxdata = rxdata;
	job->size = size;
	job->dma_state = DMA_STATE_BUSY;

	uint32_t source_address = LL_I2C_DMA_GetRegAddr(i2c_handle);
	uint32_t destination_address = (uint32_t)rxdata;
	uint32_t direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
	LL_DMA_ConfigAddresses(dma_handle, dma_stream, source_address, destination_address, direction);

	LL_DMA_SetDataLength(dma_handle, dma_stream, size);

	LL_I2C_GenerateStartCondition(i2c_handle);
	if(wait_for_condition(i2c_handle, flag_sb) != _i2c_manager_ok) goto i2c_error;

	LL_I2C_TransmitData8(i2c_handle, dev_addr & 0xFE);
	if(wait_for_condition(i2c_handle, flag_addr) != _i2c_manager_ok) goto i2c_error;
	LL_I2C_ClearFlag_ADDR(i2c_handle);

	LL_I2C_TransmitData8(i2c_handle, reg_addr);
	if(wait_for_condition(i2c_handle, flag_txe) != _i2c_manager_ok) goto i2c_error;

	LL_I2C_GenerateStartCondition(i2c_handle);
	if(wait_for_condition(i2c_handle, flag_sb) != _i2c_manager_ok) goto i2c_error;

	LL_I2C_TransmitData8(i2c_handle, dev_addr | 0x01);
	if(wait_for_condition(i2c_handle, flag_addr) != _i2c_manager_ok) goto i2c_error;

	if(size == 1){
		LL_I2C_AcknowledgeNextData(i2c_handle, LL_I2C_NACK);
	}
	else{
		LL_I2C_AcknowledgeNextData(i2c_handle, LL_I2C_ACK);
		LL_I2C_EnableLastDMA(i2c_handle);
	}

	LL_I2C_EnableIT_ERR(i2c_handle);
	LL_I2C_EnableDMAReq_RX(i2c_handle);
	LL_DMA_EnableIT_TC(dma_handle, dma_stream);
	LL_DMA_EnableIT_TE(dma_handle, dma_stream);

	LL_DMA_EnableStream(dma_handle, dma_stream);
	LL_I2C_ClearFlag_ADDR(i2c_handle);

	return _i2c_manager_ok;

			i2c_error:
			if(LL_I2C_IsActiveFlag_AF(i2c_handle)) LL_I2C_ClearFlag_AF(i2c_handle);
			job->dma_state = DMA_STATE_IDLE;
			LL_I2C_GenerateStopCondition(i2c_handle);
			osMutexRelease(job->mutexHandle);
			return _i2c_manager_timeout;
}

void i2c_manager_dma_handler(DMA_TypeDef* dma_handle, uint32_t dma_stream){
	struct dma_management* job = NULL;

	// Hangi I2C'nin kamyonu (DMA) geldi? Havuzda (Pool) arayıp buluyoruz!
	for(int i=0; i<3; i++){
		if(i2c_jobs[i].dma_state != DMA_STATE_IDLE &&
		   i2c_jobs[i].dma_handle == dma_handle &&
		   i2c_jobs[i].dma_stream == dma_stream){
			job = &i2c_jobs[i];
			break;
		}
	}

	if(job == NULL) return; // Bize ait olmayan bir DMA kesmesi ise çık

	bool transfer_complete = false;
	bool transfer_error = false;

	switch(dma_stream){
		case LL_DMA_STREAM_0:
			if(LL_DMA_IsActiveFlag_TC0(dma_handle))      { transfer_complete = true; LL_DMA_ClearFlag_TC0(dma_handle); }
			else if(LL_DMA_IsActiveFlag_TE0(dma_handle)) { transfer_error = true;    LL_DMA_ClearFlag_TE0(dma_handle); }
			break;
		case LL_DMA_STREAM_1:
			if(LL_DMA_IsActiveFlag_TC1(dma_handle))      { transfer_complete = true; LL_DMA_ClearFlag_TC1(dma_handle); }
			else if(LL_DMA_IsActiveFlag_TE1(dma_handle)) { transfer_error = true;    LL_DMA_ClearFlag_TE1(dma_handle); }
			break;
		case LL_DMA_STREAM_2:
			if(LL_DMA_IsActiveFlag_TC2(dma_handle))      { transfer_complete = true; LL_DMA_ClearFlag_TC2(dma_handle); }
			else if(LL_DMA_IsActiveFlag_TE2(dma_handle)) { transfer_error = true;    LL_DMA_ClearFlag_TE2(dma_handle); }
			break;
		case LL_DMA_STREAM_3:
			if(LL_DMA_IsActiveFlag_TC3(dma_handle))      { transfer_complete = true; LL_DMA_ClearFlag_TC3(dma_handle); }
			else if(LL_DMA_IsActiveFlag_TE3(dma_handle)) { transfer_error = true;    LL_DMA_ClearFlag_TE3(dma_handle); }
			break;
		case LL_DMA_STREAM_4:
			if(LL_DMA_IsActiveFlag_TC4(dma_handle))      { transfer_complete = true; LL_DMA_ClearFlag_TC4(dma_handle); }
			else if(LL_DMA_IsActiveFlag_TE4(dma_handle)) { transfer_error = true;    LL_DMA_ClearFlag_TE4(dma_handle); }
			break;
		case LL_DMA_STREAM_5:
			if(LL_DMA_IsActiveFlag_TC5(dma_handle))      { transfer_complete = true; LL_DMA_ClearFlag_TC5(dma_handle); }
			else if(LL_DMA_IsActiveFlag_TE5(dma_handle)) { transfer_error = true;    LL_DMA_ClearFlag_TE5(dma_handle); }
			break;
		case LL_DMA_STREAM_6:
			if(LL_DMA_IsActiveFlag_TC6(dma_handle))      { transfer_complete = true; LL_DMA_ClearFlag_TC6(dma_handle); }
			else if(LL_DMA_IsActiveFlag_TE6(dma_handle)) { transfer_error = true;    LL_DMA_ClearFlag_TE6(dma_handle); }
			break;
		case LL_DMA_STREAM_7:
			if(LL_DMA_IsActiveFlag_TC7(dma_handle))      { transfer_complete = true; LL_DMA_ClearFlag_TC7(dma_handle); }
			else if(LL_DMA_IsActiveFlag_TE7(dma_handle)) { transfer_error = true;    LL_DMA_ClearFlag_TE7(dma_handle); }
			break;
		default:
			return; // Tanımsız bir stream geldiyse çık
	}

	if(transfer_complete){
		LL_I2C_GenerateStopCondition(job->i2c_handle);
		i2c_dma_cleanup(job);
		job->dma_state = DMA_STATE_IDLE;
		osSemaphoreRelease(job->semaphoreHandle); // YENİ: Sadece İlgili Hattın Zilini Çal
	}
	else if(transfer_error){
		LL_I2C_GenerateStopCondition(job->i2c_handle);
		i2c_dma_cleanup(job);
		job->dma_state = DMA_STATE_IDLE;
		osSemaphoreRelease(job->semaphoreHandle);
	}
}

void i2c_manager_error_handler(I2C_TypeDef* i2c_handle){
	struct dma_management* job = get_job(i2c_handle);
	if(job == NULL || job->dma_state == DMA_STATE_IDLE) return;

	if(LL_I2C_IsActiveFlag_AF(job->i2c_handle)) LL_I2C_ClearFlag_AF(job->i2c_handle);
	if(LL_I2C_IsActiveFlag_BERR(job->i2c_handle)) LL_I2C_ClearFlag_BERR(job->i2c_handle);

	LL_I2C_GenerateStopCondition(job->i2c_handle);
	i2c_dma_cleanup(job);

	job->dma_state = DMA_STATE_IDLE;
	osSemaphoreRelease(job->semaphoreHandle);
}
