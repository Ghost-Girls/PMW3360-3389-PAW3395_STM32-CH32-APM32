/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f0xx_hal.h"
#include "usbd_hid.h"
#include "bsp_internal_Flash.h"
#include "mouse-for-stmf0x2.h"
#include "srom_3360_0x05.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*
mouse Byte 1 have 8bit: 00011111
00000001 = 0x01 = left;
00000010 = 0x02 = right;
00000100 = 0x04 = middle;
00001000 = 0x08 = back;
00010000 = 0x10 = forward;
*/

#define MOUSE_LEFT    0x04	//PA0 //3
#define MOUSE_RIGHT   0x08	//PA1 //4
#define MOUSE_MIDDLE  0x10	//PA2 //5
#define MOUSE_BACK		0x02	//PA3	//2
#define MOUSE_FORWARD 0x01	//PB4 //1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static unsigned char btn_keys[5] = {MOUSE_LEFT, MOUSE_RIGHT , MOUSE_MIDDLE , MOUSE_BACK , MOUSE_FORWARD};
static unsigned char b_buttons = 0;
static unsigned char b_buttons_prev = 0x00;
static unsigned char old_profile; 
static unsigned char cpi_state;
static unsigned char polling_rate_state;
static unsigned char skip;

// use this instead of bitshifts or LSB/MSB macros.
union motion_data
{
	int16_t sum;
	struct { uint8_t low, high; };
};
union motion_data x, y;
int8_t whl;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
extern USBD_HandleTypeDef hUsbDeviceFS;

//SPI_HandleTypeDef hspi1; // for HAL SPI lib

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void sensor_config(void) // Sensor Power-up configure
{
  // dpi settings CPI/CPI
	uint8_t dpi;
	
	old_profile = ((*(__IO uint16_t*)(0x08007000)) < 0xFF) ? *(__IO uint16_t*)(0x08007000) : 0x00;
	if (old_profile == 0x00)
		dpi = 0x07;// dpi = (1 + i) * 100
	if (old_profile == 0x01)
		dpi = 0x03;// dpi = (1 + i) * 100
	if (old_profile == 0x02)
		dpi = 0x0f;// dpi = (1 + i) * 100

  // Init 3360
  pmw3360_init(dpi);

  // Angle snapping settings // turn off thx.
  uint8_t angle_index = 0;
  uint8_t angles[] = {0x00, 0x80}; // Off, On
  // Init angle snapping
  angle_init(angles[angle_index]);
}
void cpi_change(uint8_t profile)	//mouse dpi change
{
		if (((btn_state[0] && btn_state[1] && btn_state[4]) == 0) && (cpi_state == 1)) // release R/B/F 3 button change cpi
		{
			cpi_state = 0;
			if (profile == 0)
			{
				old_profile = 1;
				uint8_t dpi = 0x07;
				pmw3360_set_dpi(old_profile,dpi);
			}
			else if (profile == 1)
			{
				old_profile = 2;
				uint8_t dpi = 0x0f;
				pmw3360_set_dpi(old_profile,dpi);
			}
			else if (profile == 2)
			{
				old_profile = 0;
				uint8_t dpi = 0x03;
				pmw3360_set_dpi(old_profile,dpi);
			}
		}
	else if ((btn_state[0] && btn_state[1] && btn_state[4]) != 0) // press R/B/F 3 button, updata cpi_state.
	{
		cpi_state = 1;
	}
}

static void pmw3360_set_dpi(uint16_t profile,uint8_t dpi) // mouse dpi set
{
	SS_LOW;
	spi_write(0x0f, dpi);
	spi_write(0x50, 0x00); // return to burst_motion mode
	SS_HIGH;
	InternalFlash_Write(profile);// write dpi profile to Flash.
}

uint8_t wheel_encoder_input(void) // read wheel sroll
{
// Encoder connect in circuit, i/o pin must be pullup input;		
// Encoder Pin1 --> GND/COM
// Encoder Pin2 --> Encoder_B
// Encoder Pin3 --> Encocer_A

		whl = 0; // scrolls state for usb transmission
		int8_t _whl = 0; //

		static uint8_t whl_prev_same = 0; // what A was the last time A == B
		static uint8_t whl_prev_diff = 0; // what A was the last time A != B		
		const uint8_t whl_a = WHL_A_IS_HIGH; 	//PA13
		const uint8_t whl_b = WHL_B_IS_HIGH;	//PA14

		// calculate number of scrolls
		if (whl_a != whl_b)
			whl_prev_diff = whl_a;
		else if (whl_a != whl_prev_same)
		{
			_whl = 2 * (whl_a ^ whl_prev_diff) - 1;
			whl -= _whl;
			// whl -= _whl£ºsroll forward up, sroll back down;
			// whl += _whl£ºsroll forward down, sroll back up;
			whl_prev_same = whl_a;
		}
		return whl;
	// wheel state:
	// 1: 0x01 -> sroll up;
	// 2: 0xff -> sroll down;
	// 3: 0x80 -> do not thing;
}
void polling_rate_change(uint16_t config)
{
		// release L/R/B/F 4 button, ready to change polling rate
		if (((btn_state[0] && btn_state[1] && btn_state[3]&& btn_state[4]) == 0) && (polling_rate_state == 1)) 
		{
			polling_rate_state = 0;
			if (config == 0) 			// 1000hz
			{
				skip = 1;
				Internal_Flash_Write(skip,0x08007400);
			}
			else if (config == 1) // 500Hz
			{
				skip = 3;
				Internal_Flash_Write(skip,0x08007400);
			}
			else if (config == 3) // 250hz
			{
				skip = 7;
				Internal_Flash_Write(skip,0x08007400);
			}
			else if (config == 7)	// 125hz
			{
				skip = 0;
				Internal_Flash_Write(skip,0x08007400);
			}
		}

	// press L/R/B/F 4 button, ready to updata cpi_state.
	else if ((btn_state[0] && btn_state[1] && btn_state[3] && btn_state[4]) != 0) 
	{
		polling_rate_state = 1;
	}
}
void burst_read(void)	// read sensor move
{		
		union motion_data delta_x, delta_y;
		SS_LOW;// step 1: low NCS/SS/CS Signal
		SPI_Send(0x50); // step 2: send Motion regsiter addr: 0x50.
    HAL_Delay_us(35); //step 4: wait Tsard_motbr.
		
		// at here send any vule to sensor reg
		(void) SPI_Send(0x00); // motion, not used
		(void) SPI_Send(0x00); // observation, not used
//    int motion = (SPI_Send(0x00); & 0x80) > 0;
//    int surface = (SPI_Send(0x00); & 0x08) > 0;   // 0 if on surface / 1 if off surface

		delta_x.low=SPI_Send(0x00);
		delta_x.high=SPI_Send(0x00);
		delta_y.low=SPI_Send(0x00);
		delta_y.high=SPI_Send(0x00);

//    int squal = SPI_Send(0x00);
		SS_HIGH;
 //   HAL_Delay_us(1); 
	
		x.sum += delta_x.sum;
		y.sum += delta_y.sum;
}
void Mouse_Send(void)
{
  uint8_t Mouse_Buffer[6] = {0, 0, 0, 0, 0, 0};
    
  Mouse_Buffer[0] = b_buttons;	// button  [0]-left  [1]-right [2]:middle [4]-back [5]:forward
  Mouse_Buffer[1] = x.low;	
  Mouse_Buffer[2] = x.high;	
  Mouse_Buffer[3] = y.low;	
	Mouse_Buffer[4] = y.high;	
	Mouse_Buffer[5] = whl;	// wheel sroll
	
	USBD_HID_SendReport(&hUsbDeviceFS, Mouse_Buffer,sizeof(Mouse_Buffer));
	
	b_buttons_prev = b_buttons;	// updata b_buttons_prev
	// clear movement data for next transfer.
	x.sum = 0; 
	y.sum = 0;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  MX_SPI1_Init();
	LL_SPI_Enable(SPI1);
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
	
	// PMW3360 Initial profile
	sensor_config();

// 0:1000Hz:1
// 1:500Hz:2
// 3:250Hz:4
// 7:125Hz:8
// ff:1000Hz:0:defult
	skip = ((*(__IO uint16_t*)(0x08007400)) < 0xFF) ? *(__IO uint16_t*)(0x08007400) : (HID_FS_BINTERVAL-1);
	unsigned char count = 0; // counter to skip reports

	*( unsigned int * )0x40005C40 |= 1 << 9 ;		// USB_CNTR£¬USB¿ØÖÆÆ÷ÖÐ¶ÏÆÁ±Î¼Ä´æÆ÷£¬SOFMÎ»
	*( unsigned int * )0x40005C40 |= 0 << 8 ;		// USB_CNTR£¬USBÖÐ¶Ï×´Ì¬¼Ä´æÆ÷£¬ESOFMÎ»
	*( unsigned int * )0x40005C40 |= 0 << 11 ;	// USB_CNTR£¬USB¿ØÖÆÆ÷ÖÐ¶ÏÆÁ±Î¼Ä´æÆ÷£¬SUSPMÎ»
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	*( unsigned int * )0x40005C44 |= 0 << 9 ;			// USB_ISTR£¬USBÖÐ¶Ï×´Ì¬¼Ä´æÆ÷£¬SOFÎ»£¬rc_w0(Ð´0ÓÐÐ§)
//	*( unsigned int * )0x40005C44 |= 0 << 8 ;		// USB_ISTR£¬USBÖÐ¶Ï×´Ì¬¼Ä´æÆ÷£¬ESOFÎ»
//	*( unsigned int * )0x40005C44 |= 0 << 11 ;	// USB_ISTR£¬USBÖÐ¶Ï×´Ì¬¼Ä´æÆ÷£¬SUSPMÎ»

		__WFI(); // wait about 500us for SOF interrupt to Wake-up,
		HAL_Delay_us(375); // delay time depends on your process speed.

		// Button stuff
		check_buttons_state();// get button state for usb transmission
		
		// dpi change
		cpi_change(old_profile);
		// polling rate change
		polling_rate_change(skip);
		
		// Wheel_Encoder stuff
		wheel_encoder_input(); // get scrolls state for usb transmission

		// read move data at the end to sync USB frames.
		burst_read(); 

		// skip transmission for polling rate
		if (count > 0)
		{
			count--;
			continue;
		}

		if((b_buttons_prev != b_buttons)|| x.sum || y.sum || whl)
		{
				Mouse_Send();
				count = skip;
		}
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1)
  {
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLL_MUL_6, LL_RCC_PREDIV_DIV_1);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_SetSystemCoreClock(48000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_SetUSBClockSource(LL_RCC_USB_CLKSOURCE_PLL);
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SPI1);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**SPI1 GPIO Configuration
  PA5   ------> SPI1_SCK
  PA6   ------> SPI1_MISO
  PA7   ------> SPI1_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_HIGH;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV4;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 7;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_DisableNSSPulseMgt(SPI1);
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_1);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_13;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_14;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/*************** Mouse Buttom Function ***************/
void check_buttons_state()
{
  // Fast Debounce (works with 0 latency most of the time)
  for (int i = 0; i < 5 ; i++)
  {
    /***************for M0 TSSOP-20 Package ***************/
    unsigned char state = buttons_scan(i);
// button press down, Pin low, buttong_scan return 0;		

    btn_buffers[i] = btn_buffers[i] << 1 | state; // if key HIGH buttons_scan return 0x01, else LOW return 0x00

    if (!btn_state[i] && btn_buffers[i] == 0xFE) // button pressed for the first time
    {
      press(btn_keys[i]);
      btn_state[i] = true;
    }
    else if ( (btn_state[i] && btn_buffers[i] == 0x01) // button released after stabilized press
              // force release when consequent off state (for the DEBOUNCE time) is detected
              || (btn_state[i] && btn_buffers[i] == 0xFF) )
    {
      release(btn_keys[i]);
      btn_state[i] = false;
    }
  }
}

void press(uint8_t b)
{
  Buttons(b_buttons | b);
}

void release(uint8_t b)
{
  Buttons(b_buttons & ~b);
}

uint32_t buttons_scan(unsigned char i)
{
	if ((GPIOA->IDR & 1 << (i)) != 0)
	{
		return 1;
	}
  else
  {
		return 0;
	}
}

void Buttons(uint8_t b)
{
  if (b != b_buttons)
  {
    b_buttons = b;
  }
}

/*************** PMW3360-DM Sensor Initialization ***************/

// dpi argument is what's written to register 0x0f
// actual dpi value = (dpi + 1) * 100
static void pmw3360_init(const uint8_t dpi)
{
  SS_HIGH;
  HAL_Delay(3);

  SS_LOW;	
  spi_write(0x3b, 0xb6);
	SS_HIGH;
  HAL_Delay(300);  // when power up shut down 300ms

  // drop and raise ncs to reset spi port
  SS_LOW;
  HAL_Delay_us(40);
	SS_HIGH;
  HAL_Delay_us(40);
	
	// power up reset
  SS_LOW;
  spi_write(0x3a, 0x5a);
	SS_HIGH;
  HAL_Delay(50);

  // read from 0x02 to 0x06
  SS_LOW;
  spi_read(0x02); //Motion
  spi_read(0x03); //X_lo
  spi_read(0x04); //X_Hi
  spi_read(0x05); //Y_Lo
  spi_read(0x06); //Y_Hi

  // srom download
  spi_write(0x10, 0x00);
  spi_write(0x13, 0x1d);
  SS_HIGH;
  HAL_Delay(10);
  SS_LOW;
  spi_write(0x13, 0x18);

  SPI_Send(0x62 | 0x80);
  for (uint16_t i = 0; i < SROM_LENGTH; i++)
  {
    HAL_Delay_us(16);
    SPI_Send(srom[i]);
  }
  HAL_Delay_us(18);
  SS_HIGH;
  HAL_Delay_us(200);

  // configuration/settings
  SS_LOW;
  spi_write(0x10, 0x00); // Rest mode & independant X/Y CPI disabled
  spi_write(0x0d, 0x00); // Camera angle
  spi_write(0x11, 0x00); // Camera angle fine tuning
  spi_write(0x0f, dpi); // DPI
  // LOD Stuff
  spi_write(0x63, 0x03); // LOD: 0x00 disable lift detection, 0x02 = 2mm, 0x03 = 3mm
  spi_write(0x2b, 0x80); // Minimum SQUAL for zero motion data (default: 0x10)£¬max is 0x80
  spi_write(0x2c, 0x0a); // Minimum Valid features (reduce SQUAL score) (default: 0x0a)
  SS_HIGH;
  HAL_Delay_us(200);
}

// angle snapping
static void angle_init(const uint8_t angle)
{
  SS_LOW;
  spi_write(0x42, angle); // Angle snapping (Angle snap disabled. This is the default value.): 0x00 = off, 0x80 = on
  SS_HIGH;
}


void HAL_Delay_us(__IO uint32_t delay_us)
{
  uint32_t first_value = 0;
  uint32_t current_value = 0;
  uint32_t reload = SysTick ->LOAD;

  uint32_t nus_number = delay_us * ((reload + 1) / 1000);
  uint32_t change_number = 0;

  first_value = SysTick ->VAL;
  while (1)
  {
    current_value = SysTick ->VAL;
    if (current_value != first_value)
    {

      if (current_value < first_value)
      {
        change_number += first_value - current_value;
        //change_number = first_value - current_value + change_number;
      }

      else
      {
        change_number += reload - current_value + first_value;
      }
      first_value = current_value;
      if (change_number >= nus_number)
      {
        break;
      }
    }
  }
}

//SPI Read sensor.
static inline void spi_write(const unsigned char addr, const unsigned char data)
{
  SPI_Send(addr | 0x80);
  SPI_Send(data);
  HAL_Delay_us(180); // maximum of t_SWW, t_SWR
}

static inline  const unsigned char spi_read(const unsigned char addr)
{
  SPI_Send(addr);
	HAL_Delay_us(160); // t_SRAD
  uint8_t data = SPI_Send(0x00); 
  HAL_Delay_us(20);
  return data;
}


// SPI latency depends on your circuit layout and code.
unsigned char SPI_Send(unsigned char Txdata)
{
	/*
	1. Data into Buffer.
	1. Transmit data to PMW3360.
	2. Return Receive data from Inbox.
	*/
	
	uint8_t Rxdata;
	spi_transmit_receive(Txdata,&Rxdata); 
//	Rxdata = SPI1_ReadWriteByte(Txdata);
	return Rxdata;
}
// data_in:data for transfer
// data_out: receive data
static uint8_t spi_transmit_receive(uint8_t data_in, uint8_t *data_out) 
{
//	int state = 0;
	*data_out = 0;
	uint32_t timeout_cnt;
//	static const uint32_t timeout_cnt_num;

	// Wait until TXE flag is set to send data
	timeout_cnt = 0;
	static const uint32_t timeout_cnt_num_send = 100;
	while(!LL_SPI_IsActiveFlag_TXE(SPI1)) //Tx·Ç¿Õ£¬ÓÐÊý¾Ý
	{
		timeout_cnt ++;
		if((timeout_cnt > timeout_cnt_num_send)||(LL_SPI_IsActiveFlag_TXE(SPI1)))	//Tx¿Õ£¬ÓÐÊý¾Ý
		{
//			state = -1;
			break;
		}
	}

	// Transmit data in 8 Bit mode
	LL_SPI_TransmitData8(SPI1, data_in);

	// Check BSY flag
	timeout_cnt = 0;
	static const uint32_t timeout_cnt_num_busy = 50;
	while(LL_SPI_IsActiveFlag_BSY(SPI1))
	{
		timeout_cnt ++;
		if((timeout_cnt > timeout_cnt_num_busy)||(!LL_SPI_IsActiveFlag_BSY(SPI1)))
		{
//			state = -1;
			break;
		}
	}

	// Check RXNE flag
	timeout_cnt = 0;
	static const uint32_t timeout_cnt_num_recv = 200;
	while(!LL_SPI_IsActiveFlag_RXNE(SPI1)) //Rx¿Õ
	{
		timeout_cnt ++;
		if((timeout_cnt > timeout_cnt_num_recv)||(!LL_SPI_IsActiveFlag_RXNE(SPI1)))//Rx·Ç¿Õ
		{
//			state = -1;
			break;
		}
	}

	// Read 8-Bits in the data register
	*data_out = LL_SPI_ReceiveData8(SPI1);

	return *data_out;
}
uint8_t SPI1_ReadWriteByte(uint8_t TxData)  //
{
	uint8_t retry = 0;

	/* Check if Tx buffer is empty */
	while (!LL_SPI_IsActiveFlag_TXE(SPI1))
	{
		retry++;
		if(retry > 200)
			continue;
	}

	/* Write character in Data register.
	TXE flag is cleared by reading data in DR register */
	LL_SPI_TransmitData8(SPI1, TxData);
	retry = 0;

	/* Check if Rx buffer is not empty */
	while (!LL_SPI_IsActiveFlag_RXNE(SPI1))
	{
		retry++;
		if(retry > 200) continue;
	}

	/* received byte from SPI lines. */
	return LL_SPI_ReceiveData8(SPI1);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

