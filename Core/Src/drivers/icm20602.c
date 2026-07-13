/**
  * @file    icm20602.c
  * @brief   Driver for ICM-20602 6-axis IMU (SPI1, CS = PA4)
  */
#include "drivers/icm20602.h"

/* Register map */
#define ICM20602_REG_SMPLRT_DIV      0x19U
#define ICM20602_REG_CONFIG          0x1AU
#define ICM20602_REG_GYRO_CONFIG     0x1BU
#define ICM20602_REG_ACCEL_CONFIG    0x1CU
#define ICM20602_REG_ACCEL_CONFIG2   0x1DU
#define ICM20602_REG_PWR_MGMT_1      0x6BU
#define ICM20602_REG_PWR_MGMT_2      0x6CU
#define ICM20602_REG_WHO_AM_I        0x75U
#define ICM20602_REG_ACCEL_XOUT_H    0x3BU

#define ICM20602_READ_FLAG           0x80U
#define ICM20602_SPI_TIMEOUT         100U

/* Sensitivity for gyro FS = 2000 dps and accel FS = 16g (per datasheet) */
#define ICM20602_GYRO_SENS_LSB_PER_DPS   16.4f
#define ICM20602_ACCEL_SENS_LSB_PER_G    2048.0f

#define ICM20602_CS_PORT   GPIOA
#define ICM20602_CS_PIN    GPIO_PIN_4

static SPI_HandleTypeDef *icm_hspi;
static float gyro_offset_x, gyro_offset_y, gyro_offset_z;

static inline void ICM20602_CS_Low(void)
{
  HAL_GPIO_WritePin(ICM20602_CS_PORT, ICM20602_CS_PIN, GPIO_PIN_RESET);
}

static inline void ICM20602_CS_High(void)
{
  HAL_GPIO_WritePin(ICM20602_CS_PORT, ICM20602_CS_PIN, GPIO_PIN_SET);
}

static void ICM20602_WriteReg(uint8_t reg, uint8_t data)
{
  uint8_t tx[2] = { (uint8_t)(reg & 0x7FU), data };

  ICM20602_CS_Low();
  HAL_SPI_Transmit(icm_hspi, tx, sizeof(tx), ICM20602_SPI_TIMEOUT);
  ICM20602_CS_High();
}

static uint8_t ICM20602_ReadReg(uint8_t reg)
{
  uint8_t tx = (uint8_t)(reg | ICM20602_READ_FLAG);
  uint8_t rx = 0;

  ICM20602_CS_Low();
  HAL_SPI_Transmit(icm_hspi, &tx, 1, ICM20602_SPI_TIMEOUT);
  HAL_SPI_Receive(icm_hspi, &rx, 1, ICM20602_SPI_TIMEOUT);
  ICM20602_CS_High();

  return rx;
}

static void ICM20602_ReadBurst(uint8_t reg, uint8_t *buf, uint16_t len)
{
  uint8_t tx = (uint8_t)(reg | ICM20602_READ_FLAG);

  ICM20602_CS_Low();
  HAL_SPI_Transmit(icm_hspi, &tx, 1, ICM20602_SPI_TIMEOUT);
  HAL_SPI_Receive(icm_hspi, buf, len, ICM20602_SPI_TIMEOUT);
  ICM20602_CS_High();
}

uint8_t ICM20602_ReadWhoAmI(void)
{
  return ICM20602_ReadReg(ICM20602_REG_WHO_AM_I);
}

bool ICM20602_Init(SPI_HandleTypeDef *hspi)
{
  icm_hspi = hspi;

  ICM20602_CS_High();

  /* Reset device */
  ICM20602_WriteReg(ICM20602_REG_PWR_MGMT_1, 0x80U);
  HAL_Delay(100);

  if (ICM20602_ReadWhoAmI() != ICM20602_WHO_AM_I_VALUE)
  {
    return false;
  }

  /* Wake up, select best available clock source (PLL) */
  ICM20602_WriteReg(ICM20602_REG_PWR_MGMT_1, 0x01U);
  HAL_Delay(10);

  /* Enable accel + gyro axes */
  ICM20602_WriteReg(ICM20602_REG_PWR_MGMT_2, 0x00U);

  /* DLPF: gyro bandwidth ~176Hz (Fchoice_b=00, DLPF_CFG=1) */
  ICM20602_WriteReg(ICM20602_REG_CONFIG, 0x01U);

  /* Sample rate divider: ODR = 1kHz / (1 + SMPLRT_DIV) => 1kHz */
  ICM20602_WriteReg(ICM20602_REG_SMPLRT_DIV, 0x00U);

  /* Gyro full scale = 2000 dps (FS_SEL = 3) */
  ICM20602_WriteReg(ICM20602_REG_GYRO_CONFIG, 0x18U);

  /* Accel full scale = 16g (AFS_SEL = 3) */
  ICM20602_WriteReg(ICM20602_REG_ACCEL_CONFIG, 0x18U);

  /* Accel DLPF ~218Hz bandwidth */
  ICM20602_WriteReg(ICM20602_REG_ACCEL_CONFIG2, 0x01U);

  HAL_Delay(10);

  return true;
}

void ICM20602_CalibrateGyro(uint16_t samples)
{
  uint8_t buf[14];
  int32_t sum_x = 0, sum_y = 0, sum_z = 0;

  gyro_offset_x = 0.0f;
  gyro_offset_y = 0.0f;
  gyro_offset_z = 0.0f;

  for (uint16_t i = 0; i < samples; i++)
  {
    ICM20602_ReadBurst(ICM20602_REG_ACCEL_XOUT_H, buf, sizeof(buf));

    sum_x += (int16_t)((buf[8]  << 8) | buf[9]);
    sum_y += (int16_t)((buf[10] << 8) | buf[11]);
    sum_z += (int16_t)((buf[12] << 8) | buf[13]);

    HAL_Delay(2);
  }

  if (samples > 0U)
  {
    gyro_offset_x = ((float)sum_x / (float)samples) / ICM20602_GYRO_SENS_LSB_PER_DPS;
    gyro_offset_y = ((float)sum_y / (float)samples) / ICM20602_GYRO_SENS_LSB_PER_DPS;
    gyro_offset_z = ((float)sum_z / (float)samples) / ICM20602_GYRO_SENS_LSB_PER_DPS;
  }
}

bool ICM20602_ReadData(ICM20602_Data_t *data)
{
  uint8_t buf[14];
  int16_t raw_ax, raw_ay, raw_az, raw_temp, raw_gx, raw_gy, raw_gz;

  if (data == NULL)
  {
    return false;
  }

  ICM20602_ReadBurst(ICM20602_REG_ACCEL_XOUT_H, buf, sizeof(buf));

  raw_ax   = (int16_t)((buf[0]  << 8) | buf[1]);
  raw_ay   = (int16_t)((buf[2]  << 8) | buf[3]);
  raw_az   = (int16_t)((buf[4]  << 8) | buf[5]);
  raw_temp = (int16_t)((buf[6]  << 8) | buf[7]);
  raw_gx   = (int16_t)((buf[8]  << 8) | buf[9]);
  raw_gy   = (int16_t)((buf[10] << 8) | buf[11]);
  raw_gz   = (int16_t)((buf[12] << 8) | buf[13]);

  data->ax = (float)raw_ax / ICM20602_ACCEL_SENS_LSB_PER_G;
  data->ay = (float)raw_ay / ICM20602_ACCEL_SENS_LSB_PER_G;
  data->az = (float)raw_az / ICM20602_ACCEL_SENS_LSB_PER_G;

  data->gx = ((float)raw_gx / ICM20602_GYRO_SENS_LSB_PER_DPS) - gyro_offset_x;
  data->gy = ((float)raw_gy / ICM20602_GYRO_SENS_LSB_PER_DPS) - gyro_offset_y;
  data->gz = ((float)raw_gz / ICM20602_GYRO_SENS_LSB_PER_DPS) - gyro_offset_z;

  /* Temp in degC = raw / 326.8 + 25 (per datasheet) */
  data->temp = ((float)raw_temp / 326.8f) + 25.0f;

  return true;
}
