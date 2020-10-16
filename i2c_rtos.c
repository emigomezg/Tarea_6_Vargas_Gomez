/*
 * i2c_rtos.c
 *
 *  Created on: 15 oct. 2020
 *      Author: alberto
 */

#include "i2c_rtos.h"

#include "fsl_i2c.h"
#include "fsl_clock.h"
#include "fsl_port.h"

#include "FreeRTOS.h"
#include "semphr.h"

#define NUMBER_OF_SERIAL_PORTS (3)

typedef struct
{
  uint8_t is_init;
  i2c_master_handle_t fsl_i2c_handle;
  SemaphoreHandle_t mutex_tx_rx;
  SemaphoreHandle_t tx_rx_sem;
} freertos_i2c_handle_t;

static freertos_i2c_handle_t freertos_i2c_handles[NUMBER_OF_SERIAL_PORTS] = {0};

static inline void freertos_i2c_enable_port_clock(freertos_i2c_port_t port);

static inline PORT_Type * freertos_i2c_get_port_base(freertos_i2c_port_t port);

static inline I2C_Type * freertos_i2c_get_uart_base(freertos_i2c_number_t uart_number);

static void fsl_i2c_callback(I2C_Type *base, i2c_master_handle_t *handle, status_t status, void *userData);


static void fsl_i2c_callback(I2C_Type *base, i2c_master_handle_t *handle, status_t status, void *userData)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (kStatus_Success == status)
  {
	  switch(base)
	  {
	  case I2C0:
		  xSemaphoreGiveFromISR(freertos_i2c_handles[freertos_i2c_0].tx_rx_sem, &xHigherPriorityTaskWoken);
		  break;
	  case I2C1:
		  xSemaphoreGiveFromISR(freertos_i2c_handles[freertos_i2c_1].tx_rx_sem, &xHigherPriorityTaskWoken);
		  break;
	  case I2C2:
		  xSemaphoreGiveFromISR(freertos_i2c_handles[freertos_i2c_2].tx_rx_sem, &xHigherPriorityTaskWoken);
		  break;
	  }
  }

  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

freertos_i2c_flag_t freertos_i2c_init(freertos_i2c_config_t config)
{
	freertos_i2c_flag_t retval = freertos_uart_fail;
	i2c_master_config_t fsl_i2c_config;
	port_pin_config_t freertos_i2c_config = {kPORT_PullUp, kPORT_FastSlewRate, kPORT_PassiveFilterDisable, kPORT_OpenDrainDisable, kPORT_LowDriveStrength, kPORT_MuxAlt2, kPORT_UnlockRegister,};

	if(config.i2c_number < NUMBER_OF_SERIAL_PORTS)
	{
		if(!freertos_i2c_handles[config.i2c_number].is_init)
		{
			freertos_i2c_handles[config.i2c_number].mutex_tx_rx = xSemaphoreCreateMutex();

			freertos_i2c_handles[config.i2c_number].tx_rx_sem = xSemaphoreCreateBinary();

			/* Clock Enable */
			freertos_i2c_enable_port_clock(config.i2c_number, config.port);

			/* Port Config */
			PORT_SetPinMux(freertos_i2c_get_port_base(config.port), config.scl_pin, config.pin_mux);
			PORT_SetPinMux(freertos_i2c_get_port_base(config.port), config.sda_pin, config.pin_mux);

			I2C_MasterGetDefaultConfig(&fsl_i2c_config);
			fsl_i2c_config.baudRate_Bps = config.baudrate;
			fsl_uart_config.enableTx = true;
			I2C_MasterInit(freertos_i2c_get_base(config.i2c_number), &fsl_i2c_config, CLOCK_GetFreq(kCLOCK_BusClk));

			I2C_MasterTransferCreateHandle(freertos_i2c_get_base(config.i2c_number), &freertos_i2c_handles[config.i2c_number].fsl_i2c_handle, fsl_i2c_callback, NULL);

			freertos_i2c_handles[config.uart_number].is_init = 1;

			retval = freertos_uart_sucess;
		}
	}

	return retval;
}

freertos_i2c_flag_t freertos_i2c_send(freertos_i2c_number_t uart_number, uint8_t * buffer, uint16_t lenght)
{
	freertos_i2c_flag_t flag = freertos_uart_fail;
	i2c_master_transfer_t xfer;

	if(freertos_i2c_handles[uart_number].is_init)
	{
		xfer.data = buffer;
		xfer.subaddress = subaddr;
		xfer.slaveAddress = slave_addr;
		xfer.subaddressSize = subsize;
		xfer.direction = kI2C_Write;
		xfer.flags = kI2C_TransferDefaultFlag;
		xfer.dataSize = length;

		xSemaphoreTake(freertos_i2c_handles[i2c_number].mutex_tx_rx, portMAX_DELAY);


		I2C_MasterTransferNonBlocking(freertos_i2c_get_base(i2c_number), &freertos_i2c_handles[i2c_number].fsl_i2c_handle, &xfer);

		xSemaphoreTake(freertos_i2c_handles[i2c_number].tx_rx_sem, portMAX_DELAY);


		xSemaphoreGive(freertos_i2c_handles[i2c_number].mutex_tx_rx);

		flag = freertos_i2c_sucess;
	}

	return flag;
}

