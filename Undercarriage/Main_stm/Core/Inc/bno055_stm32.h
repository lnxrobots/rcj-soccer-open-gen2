#ifndef BNO055_STM32_H_
#define BNO055_STM32_H_

#ifdef __cplusplus
  extern "C" {
#endif

//#define BNO055_I2C_ENABLED
#define BNO055_UART_ENABLED

//#include "i2c.h"

#ifdef FREERTOS_ENABLED
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#endif

#include "bno055.h"

void bno055_delay(int time) {
#ifdef FREERTOS_ENABLED
  osDelay(time);
#else
  HAL_Delay(time);
#endif
}



#ifdef BNO055_I2C_ENABLED
I2C_HandleTypeDef *_bno055_i2c_port;

void bno055_assignI2C(I2C_HandleTypeDef *hi2c_device) {
  _bno055_i2c_port = hi2c_device;
}


void bno055_writeData(uint8_t reg, uint8_t data) {
  uint8_t txdata[2] = {reg, data};
  uint8_t status;
  status = HAL_I2C_Master_Transmit(_bno055_i2c_port, BNO055_I2C_ADDR << 1,
                                   txdata, sizeof(txdata), 10);
  if (status == HAL_OK) {
    return;
  }

  if (status == HAL_ERROR) {
    printf("HAL_I2C_Master_Transmit HAL_ERROR\r\n");
  } else if (status == HAL_TIMEOUT) {
    printf("HAL_I2C_Master_Transmit HAL_TIMEOUT\r\n");
  } else if (status == HAL_BUSY) {
    printf("HAL_I2C_Master_Transmit HAL_BUSY\r\n");
  } else {
    printf("Unknown status data %d", status);
  }

  uint32_t error = HAL_I2C_GetError(_bno055_i2c_port);
  if (error == HAL_I2C_ERROR_NONE) {
    return;
  } else if (error == HAL_I2C_ERROR_BERR) {
    printf("HAL_I2C_ERROR_BERR\r\n");
  } else if (error == HAL_I2C_ERROR_ARLO) {
    printf("HAL_I2C_ERROR_ARLO\r\n");
  } else if (error == HAL_I2C_ERROR_AF) {
    printf("HAL_I2C_ERROR_AF\r\n");
  } else if (error == HAL_I2C_ERROR_OVR) {
    printf("HAL_I2C_ERROR_OVR\r\n");
  } else if (error == HAL_I2C_ERROR_DMA) {
    printf("HAL_I2C_ERROR_DMA\r\n");
  } else if (error == HAL_I2C_ERROR_TIMEOUT) {
    printf("HAL_I2C_ERROR_TIMEOUT\r\n");
  }

  HAL_I2C_StateTypeDef state = HAL_I2C_GetState(_bno055_i2c_port);
  if (state == HAL_I2C_STATE_RESET) {
    printf("HAL_I2C_STATE_RESET\r\n");
  } else if (state == HAL_I2C_STATE_READY) {
    printf("HAL_I2C_STATE_RESET\r\n");
  } else if (state == HAL_I2C_STATE_BUSY) {
    printf("HAL_I2C_STATE_BUSY\r\n");
  } else if (state == HAL_I2C_STATE_BUSY_TX) {
    printf("HAL_I2C_STATE_BUSY_TX\r\n");
  } else if (state == HAL_I2C_STATE_BUSY_RX) {
    printf("HAL_I2C_STATE_BUSY_RX\r\n");
  } else if (state == HAL_I2C_STATE_LISTEN) {
    printf("HAL_I2C_STATE_LISTEN\r\n");
  } else if (state == HAL_I2C_STATE_BUSY_TX_LISTEN) {
    printf("HAL_I2C_STATE_BUSY_TX_LISTEN\r\n");
  } else if (state == HAL_I2C_STATE_BUSY_RX_LISTEN) {
    printf("HAL_I2C_STATE_BUSY_RX_LISTEN\r\n");
  } else if (state == HAL_I2C_STATE_ABORT) {
    printf("HAL_I2C_STATE_ABORT\r\n");
  } else if (state == HAL_I2C_STATE_TIMEOUT) {
    printf("HAL_I2C_STATE_TIMEOUT\r\n");
  } else if (state == HAL_I2C_STATE_ERROR) {
    printf("HAL_I2C_STATE_ERROR\r\n");
  }
  // while (HAL_I2C_GetState(_bno055_i2c_port) != HAL_I2C_STATE_READY) {}
  // return;
}

void bno055_readData(uint8_t reg, uint8_t *data, uint8_t len) {
  HAL_I2C_Master_Transmit(_bno055_i2c_port, BNO055_I2C_ADDR << 1, &reg, 1,
                          100);
  HAL_I2C_Master_Receive(_bno055_i2c_port, BNO055_I2C_ADDR << 1, data, len,
                         100);
  // HAL_I2C_Mem_Read(_bno055_i2c_port, BNO055_I2C_ADDR_LO<<1, reg,
  // I2C_MEMADD_SIZE_8BIT, data, len, 100);
}
#endif

#ifdef BNO055_UART_ENABLED
#define BNO055_WRITEDATA_TIMEOUT 10
#define BNO055_READDATA_TIMEOUT 10
UART_HandleTypeDef *_bno055_uart_port;
int bno055_writeData_errCnt = 0;
int bno055_writeData_errCnt2 = 0;
int bno055_writeData_okCnt = 0;
int bno055_writeData_err = 0;
uint8_t bno055_TxBuffer[5];
uint8_t bno055_RxBuffer[32];

int bno055_readData_errCnt = 0;
int bno055_readData_errCnt2 = 0;
//int bno055_readData_errCnt3 = 0;
//int bno055_readData_errCnt4 = 0;
int bno055_readData_okCnt = 0;
int bno055_readData_err = 0;

int bno055_TxInProgress= 0;
int bno055_RxInProgress= 0;
int bno055_RxBuffer_len = 0;
int bno055_RxBuffer_toRead = 0;
uint8_t bno055_RxBuffer_first = 0;

void bno055_assignUART(UART_HandleTypeDef *huart_device) {
  _bno055_uart_port = huart_device;
}


void bno055_writeData(uint8_t reg, uint8_t data) {
	bno055_RxInProgress = 1;
	uint8_t status = HAL_OK;
	while(status == HAL_OK){
		status = HAL_UART_Receive(_bno055_uart_port, bno055_RxBuffer, 1, 1);
	}
	bno055_RxBuffer_len = 0;
	bno055_RxBuffer_toRead = 2;
	bno055_RxBuffer_first =  0xEE;
	status = HAL_UART_Receive_IT(_bno055_uart_port, bno055_RxBuffer, 1);
	if (status != HAL_OK) {
		bno055_writeData_err = 1;
		bno055_writeData_errCnt++;
		bno055_RxInProgress = 0;
		return;
	}
	bno055_writeData_err = 0;
	bno055_TxBuffer[0] =  0xAA;
	bno055_TxBuffer[1] =  0;
	bno055_TxBuffer[2] =  reg;
	bno055_TxBuffer[3] =  1;
	bno055_TxBuffer[4] =  data;
	bno055_TxInProgress= 1;
	status = HAL_UART_Transmit_IT(_bno055_uart_port, bno055_TxBuffer, 5);
	if (status != HAL_OK) {
		bno055_writeData_err = 1;
		bno055_writeData_errCnt++;
		bno055_TxInProgress= 0;
		HAL_UART_AbortReceive_IT(_bno055_uart_port);
		bno055_RxInProgress = 0;
		return;
	}
	int tm = 0;
	while((bno055_TxInProgress == 1) && (tm < BNO055_WRITEDATA_TIMEOUT)){
		bno055_delay(1);
		tm++;
	}
	if(bno055_TxInProgress == 1){
		HAL_UART_AbortTransmit_IT(_bno055_uart_port);
		bno055_TxInProgress = 0;
		bno055_writeData_err = 1;
		bno055_writeData_errCnt++;
		HAL_UART_AbortReceive_IT(_bno055_uart_port);
		bno055_RxInProgress = 0;
		return;
	}

	tm = 0;
	while((bno055_RxInProgress == 1) && (tm < BNO055_READDATA_TIMEOUT)){
		bno055_delay(1);
		tm++;
	}
	if(bno055_RxInProgress == 1){
		HAL_UART_AbortReceive_IT(_bno055_uart_port);
		bno055_RxInProgress = 0;
		bno055_writeData_errCnt++;
		bno055_writeData_errCnt2++;
		bno055_writeData_err = 1;
		return;
	}

	if(bno055_RxBuffer[0] != 0xEE){
		bno055_writeData_err = 1;
		bno055_writeData_errCnt++;
		return;
	}
	if(bno055_RxBuffer[1] != 1){
		bno055_writeData_err = 1;
		bno055_writeData_errCnt++;
		return;
	}
	bno055_writeData_okCnt++;
}

void bno055_readData(uint8_t reg, uint8_t *data, uint8_t len) {
	if(len+2 > sizeof(bno055_RxBuffer)){
		bno055_readData_err = 1;
		bno055_readData_errCnt++;
		*data = 0;
		return;
	}
	uint8_t status = HAL_OK;
	while(status == HAL_OK){
		status = HAL_UART_Receive(_bno055_uart_port, bno055_RxBuffer, 1, 1);
	}

	bno055_RxInProgress = 1;

	bno055_RxBuffer_len = 0;
	bno055_RxBuffer_toRead = 2+len;
	bno055_RxBuffer_first =  0xBB;
	status = HAL_UART_Receive_IT(_bno055_uart_port, bno055_RxBuffer, 1);
	if (status != HAL_OK) {
		bno055_readData_err = 1;
		bno055_readData_errCnt++;
		bno055_RxInProgress = 0;
		*data = 0;
		return;
	}
	bno055_readData_err = 0;
	bno055_TxBuffer[0] =  0xAA;
	bno055_TxBuffer[1] =  1;
	bno055_TxBuffer[2] =  reg;
	bno055_TxBuffer[3] =  len;
	bno055_TxInProgress= 1;
	status = HAL_UART_Transmit_IT(_bno055_uart_port, bno055_TxBuffer, 4);
	if (status != HAL_OK) {
		bno055_readData_err = 1;
		*data = 0;
		bno055_readData_errCnt++;
		bno055_TxInProgress = 0;
		HAL_UART_AbortReceive_IT(_bno055_uart_port);
		bno055_RxInProgress = 0;
		return;
	}
	int tm = 0;
	while((bno055_TxInProgress == 1) && (tm < BNO055_WRITEDATA_TIMEOUT)){
		bno055_delay(1);
		tm++;
	}
	if(bno055_TxInProgress == 1){
		HAL_UART_AbortTransmit_IT(_bno055_uart_port);
		bno055_TxInProgress = 0;
		bno055_readData_err = 1;
		bno055_readData_errCnt++;
		HAL_UART_AbortReceive_IT(_bno055_uart_port);
		bno055_RxInProgress = 0;
		return;
	}

	tm = 0;
	while((bno055_RxInProgress == 1) && (tm < BNO055_READDATA_TIMEOUT)){
		bno055_delay(1);
		tm++;
	}
	if(bno055_RxInProgress == 1){
		HAL_UART_AbortReceive_IT(_bno055_uart_port);
		bno055_RxInProgress = 0;
		bno055_readData_errCnt++;
		bno055_readData_errCnt2++;
		bno055_readData_err = 1;
		*data = 0;
		return;
	}
	if(bno055_RxBuffer[0] != 0xBB){
		bno055_readData_err = 1;
		*data = 0;
		bno055_readData_errCnt++;
		//bno055_readData_errCnt3++;
		return;
	}
	if(bno055_RxBuffer[1] != len){
		bno055_readData_err = 1;
		*data = 0;
		bno055_readData_errCnt++;
		//bno055_readData_errCnt4++;
		return;
	}
	memcpy(data, &bno055_RxBuffer[2], len);
	bno055_readData_okCnt++;
}
#endif

#ifdef __cplusplus
  }
#endif

#endif  // BNO055_STM32_H_
