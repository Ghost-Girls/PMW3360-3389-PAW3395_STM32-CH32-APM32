/**
  ******************************************************************************
  * @file    bsp_internalFlash.c
  * @author  fire
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   内部FLASH读写测试范例
  ******************************************************************************
  */

#include "bsp_internal_Flash.h"
#include "stm32f0xx_hal_flash.h"
#include "stm32f0xx_hal_flash_ex.h"


#define u8	unsigned char
#define u16	unsigned short
#define u32	unsigned int
#define int8_t	signed char
#define uint8_t	unsigned char
#define int16_t	short
#define uint16_t	unsigned short
#define int32_t int
#define uint32_t	unsigned int
#define __I volatile const /*!< defines 'read only' permissions */
#define __O volatile /*!< defines 'write only' permissions */
#define __IO volatile /*!< defines 'read / write' permissions */



/**
  * @brief  InternalFlash_Test,对内部FLASH进行读写测试
  * @param  None
  * @retval None
  */
void InternalFlash_Write(uint16_t Data)
{
//	uint16_t Data = 0x0001;
	uint32_t Address = 0x08007000;
	uint32_t PageError = 0;
	FLASH_EraseInitTypeDef CPI_Profile;
	CPI_Profile.TypeErase = FLASH_TYPEERASE_PAGES;
	CPI_Profile.PageAddress = Address;
	CPI_Profile.NbPages = 1;				//记录写入多少页

	HAL_FLASH_Unlock();
	HAL_FLASHEx_Erase(&CPI_Profile, &PageError);
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, Address, Data);
	FLASH_WaitForLastOperation(10);
 	HAL_FLASH_Lock();
}

void Internal_Flash_Write(uint16_t Data,uint32_t Address)
{
	uint32_t PageError = 0;
	FLASH_EraseInitTypeDef CPI_Profile;
	CPI_Profile.TypeErase = FLASH_TYPEERASE_PAGES;
	CPI_Profile.PageAddress = Address;
	CPI_Profile.NbPages = 1;				//记录写入多少页

	HAL_FLASH_Unlock();
	HAL_FLASHEx_Erase(&CPI_Profile, &PageError);
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, Address, Data);
	FLASH_WaitForLastOperation(10);
 	HAL_FLASH_Lock();
}


// HAL_FLASH_Unlock();
// HAL_FLASHEx_Erase();
// HAL_FLASH_Program();
// HAL_FLASH_Lock();