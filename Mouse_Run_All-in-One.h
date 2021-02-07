#ifndef __MOUSE_RUN_ALL_IN_ONE_H
#define __MOUSE_RUN_ALL_IN_ONE_H

//#include "stm32f0xx.h"
/* Private define ??------------------------------------------------------------*/
#define u8 unsigned char
#define u16 unsigned short
#define u32 unsigned int
#define int8_t	signed char
#define uint8_t	unsigned char
#define int16_t	short
#define uint16_t	unsigned short
#define int32_t	int
#define uint32_t	unsigned int
#define __I volatile const /*!< defines 'read only' permissions */ 
#define __O volatile /*!< defines 'write only' permissions */ 
#define __IO volatile /*!< defines 'read / write' permissions */ 


/*Buttom ??*/
// #define MOUSE_LEFT    0x01
// #define MOUSE_RIGHT   0x02
// #define MOUSE_MIDDLE  0x04
// #define MOUSE_BACK    0x08
// #define MOUSE_FORWARD 0x10

#define MOUSE_LEFT    0x01
#define MOUSE_RIGHT   0x02
#define MOUSE_MIDDLE  0x03
#define MOUSE_BACK    0x04
#define MOUSE_FORWARD 0x05

/*????*/
#define true 1
#define false 0
#define booles unsigned char

/*Wheel_Encoder ?????*/
// modify these and corresponding stuff in GPIO_Init() as needed
#define WHL_A_IS_HIGH (!!(GPIOA->IDR & (1 << 14)))   //??′??÷2ù×÷
#define WHL_B_IS_HIGH (!!(GPIOB->IDR & (1 << 1)))    //??′??÷2ù×÷


	
#define SysTick_CTRL_ENABLE_Pos             0                                             /*!< SysTick CTRL: ENABLE Position */
#define SysTick_CTRL_ENABLE_Msk            (1ul << SysTick_CTRL_ENABLE_Pos)
/* SysTick Reload Register Definitions */
#define SysTick_LOAD_RELOAD_Pos             0                                             /*!< SysTick LOAD: RELOAD Position */
#define SysTick_LOAD_RELOAD_Msk            (0xFFFFFFul << SysTick_LOAD_RELOAD_Pos)        /*!< SysTick LOAD: RELOAD Mask */

// modify these and corresponding stuff in SPI_Init() as needed
#define SS_HIGH GPIO_SetBits(GPIOA,GPIO_Pin_4)
#define SS_LOW GPIO_ResetBits(GPIOA,GPIO_Pin_4)
// #define SS_Low     (GPIOA->ODR & =~(1 << 3)) //NCS is LOW before written.
// #define SS_HIGH      (GPIOA->ODR |=(1 << 3)) //NCS is HIGH aftrer written.




/* Private variables  ---------------------------------------------------------*/

/*Buttom ??*/
char btn_state[5] = { false, false, false, false, false };
char btn_keys[5] = { MOUSE_LEFT, MOUSE_RIGHT , MOUSE_MIDDLE , MOUSE_BACK , MOUSE_FORWARD};
uint8_t btn_buffers[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
unsigned char state;
void Mouse_Send(int16_t x, int16_t y, signed char wheel);
static inline void spi_write(const uint8_t addr, const uint8_t data);
static inline uint8_t spi_read(const uint8_t addr);
static void pmw3360_init(const uint8_t dpi);
static void angle_init(const uint8_t angle);
uint8_t SPI_SendReceive(uint8_t data);

void SPI1_Init(void);
char SPI_Send(char data);
unsigned char read_register(unsigned char adress);
void SPI_Receive(unsigned char *buf, long unsigned int count);

/*Buttom ??*/
void check_buttons_state(void);
void press(uint8_t b);      //??????
void release(uint8_t b);    //??????
void Buttons(uint8_t b);    //??????
uint32_t buttons_scan(unsigned char i);   //??????,??????????

void Button_GPIO_Config(void);	 


 
void SysTick_Init(void);
//void Delay_us(__IO u32 nTime);
//#define Delay_ms(x) Delay_us(100*x)	 //单位ms

void SysTick_Delay_Us( __IO uint32_t us);
void SysTick_Delay_Ms( __IO uint32_t ms);



/*Wheel_Encoder ?????*/
void Wheel_Encoder(void);

/*Sensor_BurstRead ???????*/

#endif __MOUSE_RUN_ALL_IN_ONE_H