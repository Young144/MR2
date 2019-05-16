/**
  ******************************************************************************
  * File Name          : USART.h

 Define to prevent recursive inclusion -------------------------------------*/
#ifndef __usart_H
#define __usart_H
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "robocon.h"
/* USER CODE BEGIN Includes */

#define EN_USART_RX 1
#define USART_REC_LEN  			2000 	//定义最大接收字节数 2000
#define RXBUFFERSIZE   1 //缓存大小
extern uint16_t  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符
extern uint16_t USART_RX_STA;         		//接收状态标记
extern uint16_t aRxBuffer[RXBUFFERSIZE];//HAL库USART接收Buffer

#define EN_USART2_RX 			1		//使能（1）/禁止（0）串口1接收
#define USART2TXBUFSIZE   20 //缓存大小
#define USART2RXBUFSIZE   1 //缓存大小
extern uint8_t USART2TxBuf[USART2TXBUFSIZE];
extern uint8_t USART2RxBuf[USART2RXBUFSIZE];
extern uint16_t USART2_RX_STA;       //接收状态标记

#define EN_USART6_RX 			1		//使能（1）/禁止（0）串口1接收
#define USART6TXBUFSIZE   8 //缓存大小
#define USART6RXBUFSIZE   40 //缓存大小
extern uint8_t USART6TxBuf[USART6TXBUFSIZE];
extern uint8_t USART6RxBuf[USART6RXBUFSIZE];
extern uint16_t USART6_RX_STA;       //接收状态标记

/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart8;
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

extern void Error_Handler(void);

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);
void MX_USART6_UART_Init(void);
void MX_UART7_Init(void);
void MX_UART8_Init(void);
/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
