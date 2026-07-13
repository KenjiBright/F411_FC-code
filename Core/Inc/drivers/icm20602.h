/**
  * @file    icm20602.h
  * @brief   Driver for ICM-20602 6-axis IMU (SPI1, CS = PA4)
  */
#ifndef DRIVERS_ICM20602_H
#define DRIVERS_ICM20602_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ICM20602_WHO_AM_I_VALUE   0x12U

typedef struct
{
  float ax, ay, az;   /* g */
  float gx, gy, gz;   /* deg/s */
  float temp;         /* deg C */
} ICM20602_Data_t;

bool    ICM20602_Init(SPI_HandleTypeDef *hspi);
uint8_t ICM20602_ReadWhoAmI(void);
void    ICM20602_CalibrateGyro(uint16_t samples);
bool    ICM20602_ReadData(ICM20602_Data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_ICM20602_H */
