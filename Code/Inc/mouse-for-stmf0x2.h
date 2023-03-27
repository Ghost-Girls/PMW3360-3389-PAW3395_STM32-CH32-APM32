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

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

#if 1 // Buttom stuff


#define true 1
#define false 0
#define booles unsigned char


char btn_state[5] = { false, false, false, false, false };

uint8_t btn_buffers[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
unsigned char state;

void check_buttons_state(void);
void press(uint8_t b);
void release(uint8_t b);
void Buttons(uint8_t b);
uint32_t buttons_scan(unsigned char i);

#endif


#if 1 // wheel stuff 
#define WHL_A_IS_HIGH (!!(GPIOA->IDR & (1 << 13)))
#define WHL_B_IS_HIGH (!!(GPIOA->IDR & (1 << 14)))
#endif

#if 1	// USB_Report 
void Mouse_Send(void);
#endif
#if 1 // For sensor
/* Sensor_Trans *************************************************/
static inline void spi_write(const uint8_t addr, const uint8_t data);
static inline const unsigned char spi_read(const unsigned char addr);
static void pmw3360_init(const uint8_t dpi);
static void angle_init(const uint8_t angle);

#endif

#if 1 // For MCU_SPI

#define SS_LOW			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET)
#define SS_HIGH			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET)

unsigned char SPI_Send(unsigned char Txdata);

#endif

void HAL_Delay_us(__IO uint32_t delay_us);
uint8_t wheel_encoder_input(void);
void burst_read(void);
void sensor_config(void);
void cpi_change(uint8_t profile);
static void pmw3360_set_dpi(uint16_t profile,uint8_t dpi);
static uint8_t spi_transmit_receive(uint8_t data_in, uint8_t *data_out);
uint8_t SPI1_ReadWriteByte(uint8_t TxData);