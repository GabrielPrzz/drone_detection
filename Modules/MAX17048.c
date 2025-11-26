#include "MAX17048.h"

uint8_t MAX17048_connection(I2C_HandleTypeDef* hi2c) {

	uint8_t CHIP_ID = 0;
	HAL_I2C_Mem_Read(hi2c, MAX17048_I2CADDR_DEFAULT, MAX1704X_CHIPID_REG, 1, &CHIP_ID, 1, 1000);

	if(!CHIP_ID)
		return 0;

	//return 1;
	return CHIP_ID;

}

float_t MAX17048_getVoltage(I2C_HandleTypeDef* hi2c) {
	uint8_t VCELL_buff[2];
	float_t VCELL_raw;

	HAL_I2C_Mem_Read(hi2c, MAX17048_I2CADDR_DEFAULT, MAX1704X_VCELL_REG, 1, &VCELL_buff[0], 2, 1000);

	//Primero lee el MSB, asi que el primer valor del buffer se recorre a la derecha
	VCELL_raw = (VCELL_buff[0] << 8) | (VCELL_buff[1]);
	return VCELL_raw * 78.125 / 1000000;
}

float_t MAX17048_getPercentage(I2C_HandleTypeDef* hi2c){
	uint8_t SOC_buff[2];
	float_t SOC_raw;

	HAL_I2C_Mem_Read(hi2c, MAX17048_I2CADDR_DEFAULT, MAX1704X_SOC_REG, 1, &SOC_buff[0], 2, 1000);

	SOC_raw = (SOC_buff[0] << 8) | (SOC_buff[1]);
	return SOC_raw / 256.0;
}

float_t MAX17048_getDisChargeRate(I2C_HandleTypeDef* hi2c) {
	uint8_t CRATE_buff[2];
	float_t CRATE_raw;

	HAL_I2C_Mem_Read(hi2c, MAX17048_I2CADDR_DEFAULT, MAX1704X_CRATE_REG, 1, &CRATE_buff[0], 2, 1000);

	CRATE_raw = (CRATE_buff[0] << 8) | (CRATE_buff[1]);
	return CRATE_raw * 0.208;
}
