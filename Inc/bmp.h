

#include <stdio.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"
int32_t calib_data[12];
int t_fine;
void BMP_write8(uint8_t val, uint8_t reg);
uint8_t BMP_read8(uint8_t reg);
uint16_t BMP_read16(uint8_t reg);
uint16_t BMP_read16_LE(uint8_t reg);
int16_t BMP_readS16_LE(uint8_t reg);
uint32_t BMP_read24(uint8_t reg);
int BMP_read_temp();
int BMP_read_press();
bool BMP_begin();
void BMP_sample();
void BMP_read_coefficients();
extern SPI_HandleTypeDef hspi1;
#define SPI_HANDLE &hspi1;
#define CS_PORT GPIOC
#define CS_PIN GPIO_PIN_4
#define MODE_SLEEP  0x00
#define MODE_FORCED  0x01;
#define MODE_NORMAL 0x03
#define MODE_SOFT_RESET_CODE 0xB6
#define REGISTER_RESET 0xE0
#define REGISTER_CONFIG 0xF5
#define REGISTER_CONTROL 0xF4
#define REGISTER_CHIPID  0xD0
#define REGISTER_TEMPDATA 0xFA
#define REGISTER_PRESSDATA 0xF7
#define REGISTER_DIG_T1  0x88
#define REGISTER_DIG_T2  0x8A
#define REGISTER_DIG_T3  0x8C
#define REGISTER_DIG_P1 0x8E
#define REGISTER_DIG_P2 0x90
#define REGISTER_DIG_P3 0x92
#define REGISTER_DIG_P4 0x94
#define REGISTER_DIG_P5 0x96
#define REGISTER_DIG_P6 0x98
#define REGISTER_DIG_P7 0x9A
#define REGISTER_DIG_P8 0x9C
#define REGISTER_DIG_P9 0x9E

#define CHIPID 0x58

