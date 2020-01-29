/**
 * @file retarget.c
 * @author Ian Frosst
 * @brief C standard library retargeting functions
 * @date 2020-01-18
 *
 * @copyright Copyright (c) 2020
 *
 */

#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/unistd.h>

#include "stm32f7xx.h"
#include "usart.h"

int _write(int fd, char* buf, size_t nbyte) {
  if ((buf == NULL) || (nbyte == 0)) return 0;

  if (fd == STDOUT_FILENO) {
    if (HAL_OK != HAL_UART_Transmit(&huart3, (uint8_t*)buf, nbyte, 0xFFFF)) {
      return 0;
    }
  } else if (fd == STDERR_FILENO) {
    for (size_t i = 0; i < nbyte; ++i) {
      ITM_SendChar(buf[i]);
    }
  }

  return nbyte;
}
