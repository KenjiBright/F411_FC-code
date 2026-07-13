/**
  * @file    bmp388.h
  * @brief   Driver for BMP388 barometer (I2C1, addr 0x76)
  */
#ifndef DRIVERS_BMP388_H
#define DRIVERS_BMP388_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BMP388_CHIP_ID_VALUE   0x50U

typedef struct
{
  float pressure;     /* Pa */
  float temperature;  /* deg C */
  float altitude;     /* m, relative to 1013.25 hPa sea level reference */
} BMP388_Data_t;

bool    BMP388_Init(I2C_HandleTypeDef *hi2c);
uint8_t BMP388_ReadChipId(void);
bool    BMP388_ReadData(BMP388_Data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_BMP388_H */
