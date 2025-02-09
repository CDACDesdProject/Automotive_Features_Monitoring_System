#include "bmp280.h"
#include "stm32f4xx_hal.h"
#include "stdio.h"
extern I2C_HandleTypeDef hi2c2;



// Read Calibration Data from BMP280
uint8_t bmp280_read_calibration(BMP280_HandleTypedef *bmp) {
    uint8_t calib_data[24];

    if (HAL_I2C_Mem_Read(bmp->i2c, bmp->dev_id << 1, 0x88, 1, calib_data, 24, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }

    bmp->dig_T1 = (calib_data[1] << 8) | calib_data[0];
    bmp->dig_T2 = (calib_data[3] << 8) | calib_data[2];
    bmp->dig_T3 = (calib_data[5] << 8) | calib_data[4];

    bmp->dig_P1 = (calib_data[7] << 8) | calib_data[6];
    bmp->dig_P2 = (calib_data[9] << 8) | calib_data[8];
    bmp->dig_P3 = (calib_data[11] << 8) | calib_data[10];
    bmp->dig_P4 = (calib_data[13] << 8) | calib_data[12];
    bmp->dig_P5 = (calib_data[15] << 8) | calib_data[14];
    bmp->dig_P6 = (calib_data[17] << 8) | calib_data[16];
    bmp->dig_P7 = (calib_data[19] << 8) | calib_data[18];
    bmp->dig_P8 = (calib_data[21] << 8) | calib_data[20];
    bmp->dig_P9 = (calib_data[23] << 8) | calib_data[22];

    return HAL_OK;
}

// Read Uncompensated Temperature and Pressure
HAL_StatusTypeDef bmp280_read_uncomp_data(BMP280_HandleTypedef *dev, int32_t *temp, int32_t *pres) {
    uint8_t data[6];
    uint8_t ctrl_meas = 0x27;  // Set to forced mode (1x oversampling for temp & pressure)

// 🔹 Write to BMP280_CTRL_MEAS (0xF4) to trigger a forced measurement
    if (HAL_I2C_Mem_Write(dev->i2c, dev->dev_id << 1, 0xF4, 1, &ctrl_meas, 1, HAL_MAX_DELAY) != HAL_OK) {
        printf("Error: Failed to set forced mode!\n");
        return HAL_ERROR;
    }

    // 🔹 Wait for measurement to complete (BMP280 max measurement time: ~8ms)
    HAL_Delay(10);

    // 🔹 Read 6 bytes from BMP280 starting from register 0xF7 (pressure & temp data)
    if (HAL_I2C_Mem_Read(dev->i2c, dev->dev_id << 1, 0xF7, 1, data, 6, HAL_MAX_DELAY) != HAL_OK) {
        printf("Error: Failed to read raw data from BMP280!\n");
        return HAL_ERROR;
    }

    // 🔹 Extract raw ADC values (20-bit values)
    *pres = (int32_t)((data[0] << 12) | (data[1] << 4) | (data[2] >> 4));
    *temp = (int32_t)((data[3] << 12) | (data[4] << 4) | (data[5] >> 4));

    // 🔹 Debug print
    printf("Raw Temp ADC: %ld, Raw Pres ADC: %ld\n", *temp, *pres);

    return HAL_OK;
}

// Temperature Compensation Formula
int32_t bmp280_comp_temp_32bit(BMP280_HandleTypedef *bmp, int32_t adc_temp) {
    int32_t var1, var2, T;

    var1 = ((((adc_temp >> 3) - ((int32_t)bmp->dig_T1 << 1))) * ((int32_t)bmp->dig_T2)) >> 11;
    var2 = (((((adc_temp >> 4) - ((int32_t)bmp->dig_T1)) * ((adc_temp >> 4) - ((int32_t)bmp->dig_T1))) >> 12) *
            ((int32_t)bmp->dig_T3)) >> 14;

    bmp->t_fine = var1 + var2;
    T = (bmp->t_fine * 5 + 128) >> 8;

    return T; // Temperature in °C * 100
}

// Pressure Compensation Formula
uint32_t bmp280_comp_pres_32bit(BMP280_HandleTypedef *bmp, int32_t adc_pres) {
    int64_t var1, var2, p;

    var1 = ((int64_t)bmp->t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)bmp->dig_P6;
    var2 = var2 + ((var1 * (int64_t)bmp->dig_P5) << 17);
    var2 = var2 + (((int64_t)bmp->dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)bmp->dig_P3) >> 8) + ((var1 * (int64_t)bmp->dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)bmp->dig_P1) >> 33;

    if (var1 == 0) {
        return 0;  // Avoid division by zero
    }

    p = 1048576 - adc_pres;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)bmp->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)bmp->dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)bmp->dig_P7) << 4);

    return (uint32_t)p;  // Pressure in Pa
}
