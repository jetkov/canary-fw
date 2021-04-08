#ifndef CUSTOM_BOARD_H
#define CUSTOM_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nrf_gpio.h"

#define LEDS_NUMBER    3

#define LED_START      6
#define LED_1          6
#define LED_2          7
#define LED_3          8
#define LED_STOP       8

#define LEDS_ACTIVE_STATE 0

#define LEDS_INV_MASK  LEDS_MASK

#define LEDS_LIST { LED_1, LED_2, LED_3 }

#define BSP_LED_0      LED_1
#define BSP_LED_1      LED_2
#define BSP_LED_2      LED_3

#define BUTTONS_NUMBER 1

#define BUTTON_START   19
#define BUTTON_1       19
#define BUTTON_STOP    19
#define BUTTON_PULL    NRF_GPIO_PIN_NOPULL

#define BUTTONS_ACTIVE_STATE 1

#define BUTTONS_LIST { BUTTON_1 }

#define BSP_BUTTON_0   BUTTON_1

#define LMP_VOUT_B     5
#define MCU_PM_UART_TX 29
#define MCU_PM_UART_RX 10
#define N_PM_SLEEP     11

#define I2C_SDA        12
#define I2C_SCL        13
#define SPI_CLK        14
#define SPI_MOSI       15
#define SPI_MISO       16
#define N_FLASH_CS     17

#define N_PM_RST       20
#define CHG_STAT       22
#define EN_VDD         24
#define EN_5V          25
#define EN_VREF        26
#define N_LMP_EN       27
#define PIEZO_PWM      31

#define TX_PIN_NUMBER  MCU_PM_UART_TX
#define RX_PIN_NUMBER  MCU_PM_UART_RX
#define PM_UART_HWFC   false

#ifdef __cplusplus
}
#endif

#endif // CUSTOM_BOARD_H