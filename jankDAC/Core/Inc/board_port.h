#ifndef BOARD_PORT_H_
#define BOARD_PORT_H_

#include "stm32h5xx_hal.h"

// Adjust these to match the LED on your STM32H503 board.
#define BOARD_LED_GPIO_PORT GPIOA
#define BOARD_LED_GPIO_PIN  GPIO_PIN_5

#endif
