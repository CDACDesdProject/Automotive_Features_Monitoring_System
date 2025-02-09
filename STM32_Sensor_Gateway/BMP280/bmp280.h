#ifndef __BMP280_H
#define __BMP280_H

#include "stm32f4xx_hal.h"

#define BMP280_I2C_ADDR_PRIM      0x76    // Primary I2C address
#define BMP280_I2C_ADDR_SEC       0x77    // Secondary I2C address
#define BMP280_OK 0
// Register addresses
#define BMP280_REG_CTRL_MEAS      0xF4
#define BMP280_REG_CONFIG         0xF5
#define BMP280_REG_TEMP_DATA      0xFA
#define BMP280_REG_PRESS_DATA     0xF7

// Function prototypes

typedef struct {
    I2C_HandleTypeDef *i2c;
    uint8_t dev_id;

    uint16_t dig_T1;
    int16_t dig_T2, dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

    int32_t t_fine; // Required for compensation formula
} BMP280_HandleTypedef;


HAL_StatusTypeDef bmp280_read_uncomp_data(BMP280_HandleTypedef *dev, int32_t *temp, int32_t *pres);
uint8_t bmp280_read_calibration(BMP280_HandleTypedef *bmp);
int32_t bmp280_comp_temp_32bit(BMP280_HandleTypedef *bmp, int32_t adc_temp);
uint32_t bmp280_comp_pres_32bit(BMP280_HandleTypedef *bmp, int32_t adc_pres);

#endif // __BMP280_H
