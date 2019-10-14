#ifndef __SIM868_H
#define __SIM868_H

#include "stm32f4xx_hal.h"
#include <string.h>

void Send_AT_Command(UART_HandleTypeDef *huart, const char *pData);

#endif
