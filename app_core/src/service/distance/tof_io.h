#pragma once

#include <stdint.h>

int32_t TOFIO_Init(void);
int32_t TOFIO_DeInit(void);
int32_t TOFIO_WriteReg(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length);
int32_t TOFIO_ReadReg(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length);
int32_t TOFIO_GetTick(void);
