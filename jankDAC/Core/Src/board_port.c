#include "board_port.h"
#include "bsp/board_api.h"

uint32_t board_millis(void) {
  return HAL_GetTick();
}

void board_led_write(bool state) {
  GPIO_PinState pin_state = state ? GPIO_PIN_SET : GPIO_PIN_RESET;
  HAL_GPIO_WritePin(BOARD_LED_GPIO_PORT, BOARD_LED_GPIO_PIN, pin_state);
}
