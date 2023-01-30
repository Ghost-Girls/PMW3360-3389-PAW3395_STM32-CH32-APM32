//#ifndef __MOUSE_RUN_ALL_IN_ONE_H
//#define __MOUSE_RUN_ALL_IN_ONE_H

#include "stm32f0xx.h"
#include "usbd_def.h"
/* Private define ------------------------------------------------------------*/
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



/*Buttom Configure*/
 #define MOUSE_LEFT    0x01 //PA0
 #define MOUSE_RIGHT   0x02 //PA1
 #define MOUSE_MIDDLE  0x10 //TX
 #define MOUSE_BACK    0x08	//RX
 #define MOUSE_FORWARD 0x04 //PB1

/*bool*/
#define true 1
#define false 0
#define booles unsigned char

/* Wheel_Encoder *************************************************/
// modify these and corresponding stuff in GPIO_Init() as needed

// For TSSOP-20 MCU
#if 1
#define WHL_A_IS_HIGH (!!(GPIOA->IDR & (1 << 13)))
#define WHL_B_IS_HIGH (!!(GPIOA->IDR & (1 << 14)))
#endif

#define SS_LOW			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define SS_HIGH			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)

/* Private variables  ---------------------------------------------------------*/

/* Buttom *************************************************/
char btn_state[5] = { false, false, false, false, false };
char btn_keys[5] = { MOUSE_LEFT, MOUSE_RIGHT , MOUSE_MIDDLE , MOUSE_BACK , MOUSE_FORWARD};
uint8_t btn_buffers[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
unsigned char state;

/* USB_Report *************************************************/
void Mouse_Send(uint8_t buf0,uint8_t buf1,uint8_t buf2,uint8_t buf3);

/* Sensor_Trans *************************************************/
static inline void spi_write(const uint8_t addr, const uint8_t data);
static inline const unsigned char spi_read(const unsigned char addr);
static void pmw3360_init(const uint8_t dpi);
static void angle_init(const uint8_t angle);


/*MCU_SPI *************************************************/
#if 1 // For M0 HAL
void SPI1_Init(void);
unsigned char SPI_Send(unsigned char Txdata);
unsigned char read_register(unsigned char adress);
void SPI_Receive(unsigned char *buf, long unsigned int count);
#endif


/* Buttom *************************************************/
void check_buttons_state(void);
void press(uint8_t b);
void release(uint8_t b);
void Buttons(uint8_t b);
uint32_t buttons_scan(unsigned char i);

void Button_GPIO_Config(void);


void HAL_Delay_us(__IO uint32_t delay_us);


//#endif __MOUSE_RUN_ALL_IN_ONE_H
