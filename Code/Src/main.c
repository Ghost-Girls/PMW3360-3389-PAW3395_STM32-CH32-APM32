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
	
	
	/*
  ******************************************************************************
  it can be work, but one thing that i don't know why!
	
	the mouse speed is too low, i really hope somebody can tell me why:
	the thing is :[Intellimouse_3.0 400DPI 125Hz] = [Atmega32u4 PMW3360 800DPI 1000Hz] =[STM32F042 PMW3360 1600DPI 1000Hz]
	
	you can download full project from: https://pan.baidu.com/s/1XnpNVsO6pANIwLP8uDlTRA?pwd=3378 
	part of project code post on my gayhub: https://github.com/Ghost-Girls/STM32F042-PMW3360_Mouse/
******************************************************************************
  */
	
	// NOTE: if using Keil 5 to buil this projet, select Complier V6.15, C11, C++ 17!
		// NOTE: if using Keil 5 to buil this projet, select Complier V6.15, C11, C++ 17!
			// NOTE: if using Keil 5 to buil this projet, select Complier V6.15, C11, C++ 17!

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "usb_device.h"
#include "usbd_hid.h"

#include "Mouse_Run_All-in-One.h"
#include "srom_3360_0x04.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// mouse key configure
#define MOUSE_LEFT    0x04 //PA0
#define MOUSE_RIGHT   0x08 //PA1
#define MOUSE_MIDDLE  0x04 //TX
#define MOUSE_BACK    0x10	//RX
#define MOUSE_FORWARD 0x02 //PB1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
char btn_keys[5] = {MOUSE_LEFT, MOUSE_RIGHT , MOUSE_MIDDLE , MOUSE_BACK , MOUSE_FORWARD};
unsigned char b_buttons = 0;
uint8_t b_buttons_prev = 0x00;
int dx, dy;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
extern USBD_HandleTypeDef hUsbDeviceFS;
SPI_HandleTypeDef hspi1;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Mouse_Send(uint8_t buf0,uint8_t buf1,uint8_t buf2,uint8_t buf3)
{
  uint8_t Mouse_Buffer[4] = {0, 0, 0, 0};
    
  Mouse_Buffer[0]=b_buttons;	// button  [0]-left  [1]-right [2]:middle [4]-back [5]:forward
  Mouse_Buffer[1]=buf1;	// x-axis 
  Mouse_Buffer[2]=buf2;	// y-axis
  Mouse_Buffer[3]=buf3;	// wheel sroll
	// wheel state:
	// 1: 0x01 -> sroll up;
	// 2: 0xff -> sroll down;
	// 3: 0x80 -> do not thing;
  if((b_buttons_prev != b_buttons)||(Mouse_Buffer[1] != 0) ||(Mouse_Buffer[2] != 0)||(Mouse_Buffer[3] != 0))
  {
		USBD_HID_SendReport(&hUsbDeviceFS, Mouse_Buffer,4);
	}
	b_buttons_prev = b_buttons;	//用于下次按键状态改变与数据发送的判定
	dx = 0;
	dy = 0;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	
// wheel_stuff
		uint8_t whl_prev_same = 0; // what A was the last time A == B
		uint8_t whl_prev_diff = 0; // what A was the last time A != B

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
  MX_USB_DEVICE_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
	
#if 1	// PMW3360 Initial profile
	
  // Angle snapping settings // turn off thx.
  uint8_t angle_index = 0;
  uint8_t angles[] = {0x00, 0x80}; // Off, On

  // dpi settings CPI/CPI
	uint8_t dpi_index = 0;
	uint8_t dpis[] = {0x1f};

  // Init 3360
	
  pmw3360_init(dpis[dpi_index]);

  // Init angle snapping
  angle_init(angles[angle_index]);
#endif
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
#if 1 // Button stuff
// Button
		check_buttons_state();

#endif
#if 1 // Wheel_Encoder stuff
		
// Encoder connect in circuit, i/o pin must be pullup input;		
// Encoder Pin1 --> GND/COM
// Encoder Pin2 --> Encoder_B
// Encoder Pin3 --> Encocer_A

		int8_t whl = 0; // scrolls state for usb transmission
		int8_t _whl = 0; //
		
		const uint8_t whl_a = WHL_A_IS_HIGH; 	//PA13
		const uint8_t whl_b = WHL_B_IS_HIGH;	//PA14

		// calculate number of scrolls
		if (whl_a != whl_b)
			whl_prev_diff = whl_a;
		else if (whl_a != whl_prev_same)
		{
			_whl = 2 * (whl_a ^ whl_prev_diff) - 1;
			whl -= _whl;
			// whl -= _whl：sroll forward up, sroll back down;
			// whl += _whl：sroll forward down, sroll back up;
			whl_prev_same = whl_a;
		}

#endif	
		
#if 1 // sensor stuff old_2

		SS_LOW;// step 1: low NCS/SS/CS Signal
		SPI_Send(0x50); // step 2: send Motion regsiter addr: 0x50.
    HAL_Delay_us(35); //step 4: wait Tsard_motbr.
		
		// at here send any vule to sensor reg
		SPI_Send(0x00); // motion, not used
		SPI_Send(0x00); // observation, not used
//    int motion = (SPI_Send(0x00); & 0x80) > 0;
//    int surface = (SPI_Send(0x00); & 0x08) > 0;   // 0 if on surface / 1 if off surface

    int xl = SPI_Send(0x00); 
    int xh = SPI_Send(0x00); 
    int yl = SPI_Send(0x00); 
    int yh = SPI_Send(0x00); 
//    int squal = SPI_Send(0x00); 
		
		SS_HIGH;
		
    int x = xh<<8 | xl;	// x movement: delta_x = x-low + x-high;
    int y = yh<<8 | yl;	// y movement: delta_y = y-low + y-high;

    dx += x;
    dy += y;

#endif				

		Mouse_Send(0, dx, dy, whl);
		
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/

	/*
	spi must be configure those thing:
	1. SPI FULL-	Duplex master: 2 line
	2. Frame Format: motorola
	3. Data Size: 8bit
	4. First Bit: MSB
	5. CPOL = 1 (high); CPAL = 1 (2 edge)
	6. SPI NCS/NSS Pin: Software -> OUTPUT -> Pullup
	7. stm32 do not Embedded MISO pull-up (atmega32u4 Embedded MISO pull-up), 10k Resistor pullup to sensor power.
  */
	
	hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA13 PA14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/*************** Mouse Buttom Function ***************/
void check_buttons_state()
{
  // Fast Debounce (works with 0 latency most of the time)
  for (int i = 0; i < 5 ; i++)
  {
    /***************for M0 TSSOP-20 Package ***************/
    int state;
    if (i == 4)
      state = buttons_scan(i);
    else
			state = buttons_scan(i);
// button press down, Pin low, buttong_scan return 0;		

    btn_buffers[i] = btn_buffers[i] << 1 | state; // if key HIGH digitalRead return 0x01, else LOW return 0x00

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
  /***************for M0 TSSOP-20 Package ***************/
  if (i == 4)
  {
    if ((GPIOB->IDR & 1 << 1) != 0)
		{	
			return 1;
		}
    else
    {
			return 0;
		}
  }
  else if ((GPIOA->IDR & 1 << (i)) != 0)
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
  spi_write(0x2b, 0x10); // Minimum SQUAL for zero motion data (default: 0x10)
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
  uint8_t data = SPI_Send(0x00); // ?????
  HAL_Delay_us(20);
  return data;
}
unsigned char SPI_Send(unsigned char Txdata)
{
	/*
	1. Data into Buffer.
	1. Transmit data to PMW3360.
	2. Return Receive data from Inbox.
	*/
	
	uint8_t Rxdata;
	HAL_SPI_TransmitReceive(&hspi1,&Txdata,&Rxdata,sizeof(Txdata), 10);  
	return Rxdata;
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

