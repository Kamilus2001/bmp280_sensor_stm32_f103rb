/*
 * bmp.c
 *
 *  Created on: Oct 30, 2019
 *      Author: KamilB
 */
#include "bmp.h"

void BMP_Write8(uint8_t val, uint8_t reg){
	reg = reg & ~0x80;
	uint8_t tx_buf[] = {reg, val};
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, tx_buf, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
}
uint8_t BMP_read8(uint8_t reg){
	uint8_t value;
	reg = reg |0x80;
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &reg, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi1, &value, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
	return value;
}
uint16_t BMP_read16(uint8_t reg){
	uint16_t value;
	uint8_t byte;
	reg = reg |0x80;
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &reg, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi1, &byte, 1, HAL_MAX_DELAY);
	value = byte;
	value <<= 8;
	HAL_SPI_Receive(&hspi1, &byte, 1, HAL_MAX_DELAY);
	value |= byte;
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
	return value;
}
uint32_t BMP_read24(uint8_t reg){
	uint32_t value;
	uint8_t byte;
	reg = reg |0x80;
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &reg, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi1, &byte, 1, HAL_MAX_DELAY);
	value = byte;
	value <<= 8;
	HAL_SPI_Receive(&hspi1, &byte, 1, HAL_MAX_DELAY);
	value |= byte;
	value <<= 8;
	HAL_SPI_Receive(&hspi1, &byte, 1, HAL_MAX_DELAY);
	value |= byte;
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
	return value;
}
bool BMP_begin(){
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
	BMP_Write8(MODE_SOFT_RESET_CODE, REGISTER_RESET);
	HAL_Delay(500);
	if(BMP_read8(REGISTER_CHIPID)!=	CHIPID){
		HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
			return false;
	}
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
	return true;
}
void BMP_sample(){
	/*filer and standby set */
	uint8_t config_value=0x04, control_value=0x02;
	config_value <<=5;
	config_value += (0x04<<2);
	BMP_Write8(config_value, REGISTER_CONFIG);
	/* osrs_t osrs_p and mode config */
	control_value <<= 5;
	control_value += (0x05<<2);
	control_value += 0x03;
	BMP_Write8(control_value, REGISTER_CONTROL);
}
uint16_t BMP_read16_LE(uint8_t reg){
	uint16_t temp = BMP_read16(reg);
	return (temp>>8)|(temp<<8);
}
int16_t BMP_readS16_LE(uint8_t reg){
	int16_t temp = BMP_read16_LE(reg);
	return temp;
}
void BMP_read_coefficients(){
	calib_data[0] = BMP_read16_LE(REGISTER_DIG_T1);
	calib_data[1] = BMP_readS16_LE(REGISTER_DIG_T2);
	calib_data[2] = BMP_readS16_LE(REGISTER_DIG_T3);

	calib_data[3] = BMP_read16_LE(REGISTER_DIG_P1);
	calib_data[4] = BMP_readS16_LE(REGISTER_DIG_P2);
	calib_data[5] = BMP_readS16_LE(REGISTER_DIG_P3);
	calib_data[6] = BMP_readS16_LE(REGISTER_DIG_P4);
	calib_data[7] = BMP_readS16_LE(REGISTER_DIG_P5);
	calib_data[8] = BMP_readS16_LE(REGISTER_DIG_P6);
	calib_data[9] = BMP_readS16_LE(REGISTER_DIG_P7);
	calib_data[10] = BMP_readS16_LE(REGISTER_DIG_P8);
	calib_data[11] = BMP_readS16_LE(REGISTER_DIG_P9);
}
int BMP_read_temp(){

	int var1, var2;
	int adc_t = BMP_read24(REGISTER_TEMPDATA);
	adc_t >>= 4;
	var1 =((((adc_t>>3)-((int32_t)calib_data[0]<<1)))*((int32_t)calib_data[1])) >>11;
	var2 = (((((adc_t >> 4) - ((int32_t)calib_data[0])) *
	            ((adc_t >> 4) - ((int32_t)calib_data[0]))) >>
	           12) *
	          ((int32_t)calib_data[2])) >>
	         14;

	  t_fine = var1 + var2;

	  float T = (t_fine * 5 + 128) >> 8;
	  return T/100;
}
int BMP_read_press(){
	int64_t var1, var2, p;

	  // Must be done first to get the t_fine variable set up
	  BMP_read_temp();

	  int32_t adc_P = BMP_read24(REGISTER_PRESSDATA);
	  adc_P >>= 4;

	  var1 = ((int64_t)t_fine) - 128000;
	  var2 = var1 * var1 * (int64_t)calib_data[8];
	  var2 = var2 + ((var1 * (int64_t)calib_data[7]) << 17);
	  var2 = var2 + (((int64_t)calib_data[6]) << 35);
	  var1 = ((var1 * var1 * (int64_t)calib_data[5]) >> 8) +
	         ((var1 * (int64_t)calib_data[4]) << 12);
	  var1 =
	      (((((int64_t)1) << 47) + var1)) * ((int64_t)calib_data[3]) >> 33;

	  if (var1 == 0) {
	    return 0; // avoid exception caused by division by zero
	  }
	  p = 1048576 - adc_P;
	  p = (((p << 31) - var2) * 3125) / var1;
	  var1 = (((int64_t)calib_data[11]) * (p >> 13) * (p >> 13)) >> 25;
	  var2 = (((int64_t)calib_data[10]) * p) >> 19;

	  p = ((p + var1 + var2) >> 8) + (((int64_t)calib_data[9]) << 4);
	  return p / 256;
}
