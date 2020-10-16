/*
 * BMI160.h
 *
 *  Created on: 15 oct. 2020
 *      Author: Emi
 */

#include "BMI160.h"

#define SIZE_OF_DATA	1U
#define SUBSIZE			1

typedef struct{
	rtos_i2c_number_t 	port;
	uint8_t 			bmi_addres;
}port_data_t;

port_data_t g_port;

BooleanType BMI160_Init(BMI160_config_t configuration)
{
	rtos_i2c_flag_t i2c_status = rtos_i2c_fail;
	i2c_status = rtos_i2c_init(configuration.i2c_port_config);

	if (rtos_i2c_fail == i2c_status)
	{
		return FALSE;
	}

	g_port.port = configuration.i2c_port_config.port;
	g_port.bmi_addres = configuration.base_address;

	/*accelerometer init*/
	i2c_status = rtos_i2c_transfer(
			configuration.i2c_port_config.port,
			&configuration.acc_mode,
			SIZE_OF_DATA,
			configuration.base_address,
			configuration.sub_address,
			SUBSIZE);

	if (rtos_i2c_fail == i2c_status)
	{
		return FALSE;
	}

	/*gyroscope init*/
	i2c_status = rtos_i2c_transfer(
			configuration.i2c_port_config.port,
			&configuration.gyro_mode,
			SIZE_OF_DATA,
			configuration.base_address,
			configuration.sub_address,
			SUBSIZE);

	if (rtos_i2c_fail == i2c_status)
	{
		return FALSE;
	}
	return TRUE;
}

bmi160_raw_data_t BMI160_transfer_accelerometer(void)
{
	bmi160_raw_data_t acc_data;
	uint8_t MSB = 0;
	uint8_t LSB = 0;

	//Receiving MSB and  LSB of Accelerometer X,Y and Z
	rtos_i2c_receive(g_port.port, &MSB, SIZE_OF_DATA, g_port.bmi_addres,BMI160_ACC_X_MSB_REG, SUBSIZE);
	rtos_i2c_receive(g_port.port, &LSB, SIZE_OF_DATA, BMI160_SLAVE_ADDRESS,BMI160_ACC_X_LSB_REG, SUBSIZE);
	rtos_i2c_receive(g_port.port, &MSB, SIZE_OF_DATA, g_port.bmi_addres,BMI160_ACC_Y_MSB_REG, SUBSIZE);
	rtos_i2c_receive(g_port.port, &LSB, SIZE_OF_DATA, g_port.bmi_addres,BMI160_ACC_Y_LSB_REG, SUBSIZE);
	rtos_i2c_receive(g_port.port, &MSB, SIZE_OF_DATA, g_port.bmi_addres,BMI160_ACC_Z_MSB_REG, SUBSIZE);
	rtos_i2c_receive(g_port.port, &LSB, SIZE_OF_DATA, g_port.bmi_addres,BMI160_ACC_Z_LSB_REG, SUBSIZE);

	//We concatenate data and return it
	acc_data.x = (MSB << 8) + LSB;
	acc_data.y = (MSB << 8) + LSB;
	acc_data.z = (MSB << 8) + LSB;

	return acc_data;
};


bmi160_raw_data_t BMI160_transfer_gyroscope(void)
{
	bmi160_raw_data_t gyro_data;
	uint8_t MSB = 0;
	uint8_t LSB = 0;

	//Receiving MSB and  LSB of Accelerometer X,Y and Z
	rtos_i2c_receive(g_port.port, &MSB, SIZE_OF_DATA, g_port.bmi_addres,BMI160_GYRO_X_MSB_REG, SUBSIZE);
	rtos_i2c_receive(g_port.port, &LSB, SIZE_OF_DATA, BMI160_SLAVE_ADDRESS,BMI160_GYRO_X_LSB_REG, SUBSIZE);
	rtos_i2c_receive(g_port.port, &MSB, SIZE_OF_DATA, g_port.bmi_addres,BMI160_GYRO_Y_MSB_REG, SUBSIZE);
	rtos_i2c_receive(g_port.port, &LSB, SIZE_OF_DATA, g_port.bmi_addres,BMI160_GYRO_Y_LSB_REG, SUBSIZE);
	rtos_i2c_receive(g_port.port, &MSB, SIZE_OF_DATA, g_port.bmi_addres,BMI160_GYRO_Z_MSB_REG, SUBSIZE);
	rtos_i2c_receive(g_port.port, &LSB, SIZE_OF_DATA, g_port.bmi_addres,BMI160_GYRO_Z_LSB_REG, SUBSIZE);

	//We concatenate data and return it
	gyro_data.x = (MSB << 8) + LSB;
	gyro_data.y = (MSB << 8) + LSB;
	gyro_data.z = (MSB << 8) + LSB;

	return gyro_data;
};
