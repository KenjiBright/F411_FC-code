/**
  * @file    w25q128.c
  * @brief   Driver for W25Q128 16MB SPI NOR flash (SPI2, CS = PB12)
  */
#include "drivers/w25q128.h"

/* Commands */
#define W25Q128_CMD_WRITE_ENABLE    0x06U
#define W25Q128_CMD_WRITE_DISABLE   0x04U
#define W25Q128_CMD_READ_STATUS1    0x05U
#define W25Q128_CMD_PAGE_PROGRAM    0x02U
#define W25Q128_CMD_SECTOR_ERASE    0x20U
#define W25Q128_CMD_CHIP_ERASE      0xC7U
#define W25Q128_CMD_READ_DATA       0x03U
#define W25Q128_CMD_JEDEC_ID        0x9FU

#define W25Q128_STATUS1_BUSY_BIT    0x01U

#define W25Q128_JEDEC_MANUFACTURER  0xEFU
#define W25Q128_JEDEC_MEMORY_TYPE   0x40U
#define W25Q128_JEDEC_CAPACITY      0x18U

#define W25Q128_SPI_TIMEOUT         100U
#define W25Q128_BUSY_TIMEOUT_PAGE   50U
#define W25Q128_BUSY_TIMEOUT_SECTOR 500U
#define W25Q128_BUSY_TIMEOUT_CHIP   200000U

#define W25Q128_CS_PORT   GPIOB
#define W25Q128_CS_PIN    GPIO_PIN_12

static SPI_HandleTypeDef *w25_hspi;

static inline void W25Q128_CS_Low(void)
{
  HAL_GPIO_WritePin(W25Q128_CS_PORT, W25Q128_CS_PIN, GPIO_PIN_RESET);
}

static inline void W25Q128_CS_High(void)
{
  HAL_GPIO_WritePin(W25Q128_CS_PORT, W25Q128_CS_PIN, GPIO_PIN_SET);
}

static void W25Q128_SendAddress(uint32_t addr)
{
  uint8_t addr_bytes[3];

  addr_bytes[0] = (uint8_t)(addr >> 16);
  addr_bytes[1] = (uint8_t)(addr >> 8);
  addr_bytes[2] = (uint8_t)addr;

  HAL_SPI_Transmit(w25_hspi, addr_bytes, sizeof(addr_bytes), W25Q128_SPI_TIMEOUT);
}

static uint8_t W25Q128_ReadStatus1(void)
{
  uint8_t cmd = W25Q128_CMD_READ_STATUS1;
  uint8_t status = 0;

  W25Q128_CS_Low();
  HAL_SPI_Transmit(w25_hspi, &cmd, 1, W25Q128_SPI_TIMEOUT);
  HAL_SPI_Receive(w25_hspi, &status, 1, W25Q128_SPI_TIMEOUT);
  W25Q128_CS_High();

  return status;
}

static bool W25Q128_WaitBusy(uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();

  while (W25Q128_ReadStatus1() & W25Q128_STATUS1_BUSY_BIT)
  {
    if ((HAL_GetTick() - start) > timeout_ms)
    {
      return false;
    }
  }

  return true;
}

static void W25Q128_WriteEnable(void)
{
  uint8_t cmd = W25Q128_CMD_WRITE_ENABLE;

  W25Q128_CS_Low();
  HAL_SPI_Transmit(w25_hspi, &cmd, 1, W25Q128_SPI_TIMEOUT);
  W25Q128_CS_High();
}

bool W25Q128_ReadJedecId(uint8_t *manufacturer, uint8_t *memory_type, uint8_t *capacity)
{
  uint8_t cmd = W25Q128_CMD_JEDEC_ID;
  uint8_t id[3] = { 0 };

  W25Q128_CS_Low();
  HAL_SPI_Transmit(w25_hspi, &cmd, 1, W25Q128_SPI_TIMEOUT);
  HAL_SPI_Receive(w25_hspi, id, sizeof(id), W25Q128_SPI_TIMEOUT);
  W25Q128_CS_High();

  if (manufacturer != NULL) { *manufacturer = id[0]; }
  if (memory_type  != NULL) { *memory_type  = id[1]; }
  if (capacity     != NULL) { *capacity     = id[2]; }

  return (id[0] == W25Q128_JEDEC_MANUFACTURER)
      && (id[1] == W25Q128_JEDEC_MEMORY_TYPE)
      && (id[2] == W25Q128_JEDEC_CAPACITY);
}

bool W25Q128_Init(SPI_HandleTypeDef *hspi)
{
  w25_hspi = hspi;

  W25Q128_CS_High();

  return W25Q128_ReadJedecId(NULL, NULL, NULL);
}

void W25Q128_Read(uint32_t addr, uint8_t *buf, uint32_t len)
{
  uint8_t cmd = W25Q128_CMD_READ_DATA;

  W25Q128_CS_Low();
  HAL_SPI_Transmit(w25_hspi, &cmd, 1, W25Q128_SPI_TIMEOUT);
  W25Q128_SendAddress(addr);
  HAL_SPI_Receive(w25_hspi, buf, len, HAL_MAX_DELAY);
  W25Q128_CS_High();
}

bool W25Q128_PageWrite(uint32_t addr, const uint8_t *buf, uint16_t len)
{
  uint8_t cmd = W25Q128_CMD_PAGE_PROGRAM;

  if ((len == 0U) || (len > W25Q128_PAGE_SIZE))
  {
    return false;
  }

  W25Q128_WriteEnable();

  W25Q128_CS_Low();
  HAL_SPI_Transmit(w25_hspi, &cmd, 1, W25Q128_SPI_TIMEOUT);
  W25Q128_SendAddress(addr);
  HAL_SPI_Transmit(w25_hspi, (uint8_t *)buf, len, HAL_MAX_DELAY);
  W25Q128_CS_High();

  return W25Q128_WaitBusy(W25Q128_BUSY_TIMEOUT_PAGE);
}

bool W25Q128_SectorErase(uint32_t addr)
{
  uint8_t cmd = W25Q128_CMD_SECTOR_ERASE;

  W25Q128_WriteEnable();

  W25Q128_CS_Low();
  HAL_SPI_Transmit(w25_hspi, &cmd, 1, W25Q128_SPI_TIMEOUT);
  W25Q128_SendAddress(addr);
  W25Q128_CS_High();

  return W25Q128_WaitBusy(W25Q128_BUSY_TIMEOUT_SECTOR);
}

bool W25Q128_ChipErase(void)
{
  uint8_t cmd = W25Q128_CMD_CHIP_ERASE;

  W25Q128_WriteEnable();

  W25Q128_CS_Low();
  HAL_SPI_Transmit(w25_hspi, &cmd, 1, W25Q128_SPI_TIMEOUT);
  W25Q128_CS_High();

  return W25Q128_WaitBusy(W25Q128_BUSY_TIMEOUT_CHIP);
}
