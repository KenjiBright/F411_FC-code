/**
  * @file    bmp388.c
  * @brief   Driver for BMP388 barometer (I2C1, addr 0x76)
  */
#include "drivers/bmp388.h"
#include <math.h>

/* Register map */
#define BMP388_REG_CHIP_ID     0x00U
#define BMP388_REG_DATA_0      0x04U   /* press_xlsb ... temp_msb, 6 bytes */
#define BMP388_REG_PWR_CTRL    0x1BU
#define BMP388_REG_OSR         0x1CU
#define BMP388_REG_ODR         0x1DU
#define BMP388_REG_CONFIG      0x1FU
#define BMP388_REG_CALIB_DATA  0x31U   /* 21 bytes of NVM trim coefficients */
#define BMP388_REG_CMD         0x7EU

#define BMP388_CMD_SOFTRESET   0xB6U

#define BMP388_I2C_ADDR        (0x76U << 1)  /* SDO -> GND */
#define BMP388_I2C_TIMEOUT     100U

#define BMP388_SEALEVEL_PA     101325.0f

typedef struct
{
  float par_t1, par_t2, par_t3;
  float par_p1, par_p2, par_p3, par_p4, par_p5, par_p6, par_p7, par_p8, par_p9, par_p10, par_p11;
} BMP388_QuantCalib_t;

static I2C_HandleTypeDef *bmp_hi2c;
static BMP388_QuantCalib_t calib;

static void BMP388_WriteReg(uint8_t reg, uint8_t data)
{
  HAL_I2C_Mem_Write(bmp_hi2c, BMP388_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, BMP388_I2C_TIMEOUT);
}

static uint8_t BMP388_ReadReg(uint8_t reg)
{
  uint8_t data = 0;
  HAL_I2C_Mem_Read(bmp_hi2c, BMP388_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, BMP388_I2C_TIMEOUT);
  return data;
}

static void BMP388_ReadBurst(uint8_t reg, uint8_t *buf, uint16_t len)
{
  HAL_I2C_Mem_Read(bmp_hi2c, BMP388_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, len, BMP388_I2C_TIMEOUT);
}

uint8_t BMP388_ReadChipId(void)
{
  return BMP388_ReadReg(BMP388_REG_CHIP_ID);
}

static void BMP388_ParseCalibData(void)
{
  uint8_t raw[21];
  uint16_t nvm_par_t1, nvm_par_t2, nvm_par_p5, nvm_par_p6;
  int16_t  nvm_par_p1, nvm_par_p2, nvm_par_p9;
  int8_t   nvm_par_t3, nvm_par_p3, nvm_par_p4, nvm_par_p7, nvm_par_p8, nvm_par_p10, nvm_par_p11;

  BMP388_ReadBurst(BMP388_REG_CALIB_DATA, raw, sizeof(raw));

  nvm_par_t1 = (uint16_t)(raw[0]  | (raw[1]  << 8));
  nvm_par_t2 = (uint16_t)(raw[2]  | (raw[3]  << 8));
  nvm_par_t3 = (int8_t)raw[4];
  nvm_par_p1 = (int16_t)(raw[5]  | (raw[6]  << 8));
  nvm_par_p2 = (int16_t)(raw[7]  | (raw[8]  << 8));
  nvm_par_p3 = (int8_t)raw[9];
  nvm_par_p4 = (int8_t)raw[10];
  nvm_par_p5 = (uint16_t)(raw[11] | (raw[12] << 8));
  nvm_par_p6 = (uint16_t)(raw[13] | (raw[14] << 8));
  nvm_par_p7 = (int8_t)raw[15];
  nvm_par_p8 = (int8_t)raw[16];
  nvm_par_p9 = (int16_t)(raw[17] | (raw[18] << 8));
  nvm_par_p10 = (int8_t)raw[19];
  nvm_par_p11 = (int8_t)raw[20];

  /* Quantization per Bosch BMP388 datasheet floating-point compensation */
  calib.par_t1  = (float)nvm_par_t1 / powf(2.0f, -8.0f);
  calib.par_t2  = (float)nvm_par_t2 / powf(2.0f, 30.0f);
  calib.par_t3  = (float)nvm_par_t3 / powf(2.0f, 48.0f);
  calib.par_p1  = ((float)nvm_par_p1 - powf(2.0f, 14.0f)) / powf(2.0f, 20.0f);
  calib.par_p2  = ((float)nvm_par_p2 - powf(2.0f, 14.0f)) / powf(2.0f, 29.0f);
  calib.par_p3  = (float)nvm_par_p3 / powf(2.0f, 32.0f);
  calib.par_p4  = (float)nvm_par_p4 / powf(2.0f, 37.0f);
  calib.par_p5  = (float)nvm_par_p5 / powf(2.0f, -3.0f);
  calib.par_p6  = (float)nvm_par_p6 / powf(2.0f, 6.0f);
  calib.par_p7  = (float)nvm_par_p7 / powf(2.0f, 8.0f);
  calib.par_p8  = (float)nvm_par_p8 / powf(2.0f, 15.0f);
  calib.par_p9  = (float)nvm_par_p9 / powf(2.0f, 48.0f);
  calib.par_p10 = (float)nvm_par_p10 / powf(2.0f, 48.0f);
  calib.par_p11 = (float)nvm_par_p11 / powf(2.0f, 65.0f);
}

static float BMP388_CompensateTemperature(uint32_t uncomp_temp)
{
  float partial_data1 = (float)uncomp_temp - calib.par_t1;
  float partial_data2 = partial_data1 * calib.par_t2;

  return partial_data2 + (partial_data1 * partial_data1) * calib.par_t3;
}

static float BMP388_CompensatePressure(uint32_t uncomp_press, float comp_temp)
{
  float partial_data1, partial_data2, partial_data3, partial_data4;
  float partial_out1, partial_out2;
  float up = (float)uncomp_press;

  partial_data1 = calib.par_p6 * comp_temp;
  partial_data2 = calib.par_p7 * (comp_temp * comp_temp);
  partial_data3 = calib.par_p8 * (comp_temp * comp_temp * comp_temp);
  partial_out1  = calib.par_p5 + partial_data1 + partial_data2 + partial_data3;

  partial_data1 = calib.par_p2 * comp_temp;
  partial_data2 = calib.par_p3 * (comp_temp * comp_temp);
  partial_data3 = calib.par_p4 * (comp_temp * comp_temp * comp_temp);
  partial_out2  = up * (calib.par_p1 + partial_data1 + partial_data2 + partial_data3);

  partial_data1 = up * up;
  partial_data2 = calib.par_p9 + calib.par_p10 * comp_temp;
  partial_data3 = partial_data1 * partial_data2;
  partial_data4 = partial_data3 + (up * up * up) * calib.par_p11;

  return partial_out1 + partial_out2 + partial_data4;
}

bool BMP388_Init(I2C_HandleTypeDef *hi2c)
{
  bmp_hi2c = hi2c;

  if (BMP388_ReadChipId() != BMP388_CHIP_ID_VALUE)
  {
    return false;
  }

  BMP388_WriteReg(BMP388_REG_CMD, BMP388_CMD_SOFTRESET);
  HAL_Delay(10);

  BMP388_ParseCalibData();

  /* OSR: pressure x8 (011), temperature x1 (000) */
  BMP388_WriteReg(BMP388_REG_OSR, 0x03U);

  /* ODR: 50 Hz (odr_sel = 0x02) */
  BMP388_WriteReg(BMP388_REG_ODR, 0x02U);

  /* IIR filter coefficient = 1 */
  BMP388_WriteReg(BMP388_REG_CONFIG, 0x02U);

  /* Enable pressure + temperature measurement, normal mode */
  BMP388_WriteReg(BMP388_REG_PWR_CTRL, 0x33U);

  HAL_Delay(10);

  return true;
}

bool BMP388_ReadData(BMP388_Data_t *data)
{
  uint8_t buf[6];
  uint32_t uncomp_press, uncomp_temp;
  float comp_temp;

  if (data == NULL)
  {
    return false;
  }

  BMP388_ReadBurst(BMP388_REG_DATA_0, buf, sizeof(buf));

  uncomp_press = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) | ((uint32_t)buf[2] << 16);
  uncomp_temp  = (uint32_t)buf[3] | ((uint32_t)buf[4] << 8) | ((uint32_t)buf[5] << 16);

  comp_temp = BMP388_CompensateTemperature(uncomp_temp);

  data->temperature = comp_temp;
  data->pressure = BMP388_CompensatePressure(uncomp_press, comp_temp);
  data->altitude = 44330.0f * (1.0f - powf(data->pressure / BMP388_SEALEVEL_PA, 0.1903f));

  return true;
}
