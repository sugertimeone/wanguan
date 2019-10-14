#include "SIM868.h"

void Send_AT_Command(UART_HandleTypeDef *huart, const char *pData)
{
  HAL_UART_Transmit(huart, (uint8_t *)pData, strlen(pData)+1, 1000);		
}
