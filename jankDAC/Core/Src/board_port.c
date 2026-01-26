#include "board_port.h"
#include "bsp/board_api.h"

uint32_t board_millis(void) {
  return HAL_GetTick();
}

uint32_t tusb_time_millis_api(void) {
  return board_millis();
}

void board_led_write(bool state) {
  GPIO_PinState pin_state = state ? GPIO_PIN_SET : GPIO_PIN_RESET;
  HAL_GPIO_WritePin(BOARD_LED_GPIO_PORT, BOARD_LED_GPIO_PIN, pin_state);
}

size_t board_get_unique_id(uint8_t id[], size_t max_len) {
  uint32_t uid_words[3];
  size_t copy_len = sizeof(uid_words);

  uid_words[0] = ((volatile uint32_t*) UID_BASE)[0];
  uid_words[1] = ((volatile uint32_t*) UID_BASE)[1];
  uid_words[2] = ((volatile uint32_t*) UID_BASE)[2];

  if (max_len < copy_len) {
    copy_len = max_len;
  }

  memcpy(id, uid_words, copy_len);
  return copy_len;
}
