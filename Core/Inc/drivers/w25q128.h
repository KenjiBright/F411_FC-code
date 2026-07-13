/**
  * @file    w25q128.h
  * @brief   Driver for W25Q128 16MB SPI NOR flash (SPI2, CS = PB12)
  */
#ifndef DRIVERS_W25Q128_H
#define DRIVERS_W25Q128_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define W25Q128_PAGE_SIZE     256U
#define W25Q128_SECTOR_SIZE   4096U

bool W25Q128_Init(SPI_HandleTypeDef *hspi);
bool W25Q128_ReadJedecId(uint8_t *manufacturer, uint8_t *memory_type, uint8_t *capacity);

void W25Q128_Read(uint32_t addr, uint8_t *buf, uint32_t len);
bool W25Q128_PageWrite(uint32_t addr, const uint8_t *buf, uint16_t len);
bool W25Q128_SectorErase(uint32_t addr);
bool W25Q128_ChipErase(void);

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_W25Q128_H */
