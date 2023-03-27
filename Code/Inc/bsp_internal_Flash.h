#ifndef __INTERNAL_FLASH_H
#define __INTERNAL_FLASH_H

#include "stm32f0xx_hal.h"

void InternalFlash_Write(uint16_t Data);
void Internal_Flash_Write(uint16_t Data,uint32_t Address);
#endif /* __INTERNAL_FLASH_H */

