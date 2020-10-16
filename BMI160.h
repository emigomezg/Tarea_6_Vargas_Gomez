/*
 * BMI160.h
 *
 *  Created on: 15 oct. 2020
 *      Author: Emi
 */

#ifndef BMI160_H_
#define BMI160_H_

#include <stdint.h>
#include "Bits.h"
#include "i2c_rtos.h"

#define BMI160_SLAVE_ADDRESS 	0x68
//BMI160 Address
#define BMI160_CMD_REGISTER 	0x7E
//PMU REGISTER
#define BMI160_ACC_NORMAL_MODE 	0x11
//CMD_REG = ACCELEROMETER NORMAL MODE VALUE
#define BMI160_GYRO_NORMAL_MODE 0x15
//CMD_REG = GYROSCOPE NORMAL MODE VALUE

//X coordinate for gyroscope
#define BMI160_GYRO_X_LSB_REG 	0x0C
#define BMI160_GYRO_X_MSB_REG 	0x0D

//Y coordinate for gyroscope
#define BMI160_GYRO_Y_LSB_REG 	0x0E
#define BMI160_GYRO_Y_MSB_REG 	0x0F

//Z coordinate for gyroscope
#define BMI160_GYRO_Z_LSB_REG 	0x10
#define BMI160_GYRO_Z_MSB_REG 	0x11

//X coordinate for accelerometer
#define BMI160_ACC_X_LSB_REG 	0x12
#define BMI160_ACC_X_MSB_REG 	0x13

//Y coordinate for accelerometer
#define BMI160_ACC_Y_LSB_REG 	0x14
#define BMI160_ACC_Y_MSB_REG 	0x15

//Z coordinate for accelerometer
#define BMI160_ACC_Z_LSB_REG 	0x16
#define BMI160_ACC_Z_MSB_REG 	0x17

typedef struct{

	uint8_t 			base_address;
	uint8_t				sub_address;
	uint8_t 			acc_mode;
	uint8_t 			gyro_mode;
	rtos_i2c_config_t 	i2c_port_config;

}BMI160_config_t;

typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
}bmi160_raw_data_t;

BooleanType BMI160_Init(BMI160_config_t configuration);

bmi160_raw_data_t BMI160_transfer_accelerometer(void);

bmi160_raw_data_t BMI160_transfer_gyroscope(void);

#endif /* BMI160_H_ */
