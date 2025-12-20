#ifndef __FLASH_H
#define __FLASH_H
#include "stdint.h"
#include "stm32f1xx_hal.h"
void FLASH_Lock();
void FLASH_UnLock();
void FLASHEx_Erase(uint32_t addr);
void FLash_write(uint32_t addr, uint8_t *mdata,uint16_t len);



#endif