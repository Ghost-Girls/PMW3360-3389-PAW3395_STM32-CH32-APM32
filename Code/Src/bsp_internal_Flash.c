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
void Internal_Flash_Write(uint32_t Page_Address,uint16_t Data)
{
	uint32_t PageError = 0;
	FLASH_EraseInitTypeDef Profile;
	Profile.TypeErase = FLASH_TYPEERASE_PAGES;
	Profile.PageAddress = Page_Address;
	Profile.NbPages = 1;				//记录写入多少页

	HAL_FLASH_Unlock();
	HAL_FLASHEx_Erase(&Profile, &PageError);
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, Page_Address, Data);
	FLASH_WaitForLastOperation(10);
 	HAL_FLASH_Lock();
}


// HAL_FLASH_Unlock();
// HAL_FLASHEx_Erase();
// HAL_FLASH_Program();
// HAL_FLASH_Lock();