/**
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 * @file       bsp_uart.h
 * @brief      this file contains the common defines and functions prototypes for
 *             the bsp_uart.c driver
 * @note
 * @Version    V1.0.0
 * @Date       Jan-30-2018
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 */

#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#include "robocon.h"

#define UART_RX_DMA_SIZE (1024)


#define USART2_MAX_LEN     (50)
#define USART2_BUFLEN      (1)
#define USART2_HUART       huart2

#define USART3_MAX_LEN     (50)
#define USART3_BUFLEN      (16)
#define USART3_HUART       huart3

#define IMU_MAX_LEN     (50)
#define IMU_BUFLEN      (28)
#define IMU_HUART       huart6

#define OPENMV_MAX_LEN     (50)
#define OPENMV_BUFLEN      (16)
#define OPENMV_HUART       huart7



extern float _Pitch_initial,_Roll_initial,_Pitch_rev,_Roll_rev;


extern uint8_t   imu_buf[IMU_BUFLEN];
extern uint8_t   openmv_buf[OPENMV_BUFLEN];

extern uint8_t   usart3_buf[USART3_BUFLEN];
extern uint8_t   usart2_buf[USART2_BUFLEN];
extern float yaw_calibrated;
extern bool imu_rec_flag;
typedef union _imu_data
{
    uint8_t data[24];
    float ActVal[6];
} imudata;

extern imudata imuinfo ;


typedef union _openmv_data
{
    uint8_t data[12];
    float ActVal[3];
} openmvdata;

extern openmvdata openmvinfo ;
extern openmvdata openmv2info ;

typedef struct _ps2_data
{
    uint8_t KEY_VALUE;
    uint8_t PSS_LX_VALUE;
    uint8_t PSS_LY_VALUE;
    uint8_t PSS_RX_VALUE;
    uint8_t PSS_RY_VALUE;

} ps2data;

extern ps2data ps2info;


void uart_receive_handler(UART_HandleTypeDef *huart);
void imu_uart_init(void);
void openmv_uart_init(void);
void uart_receive_init(UART_HandleTypeDef *huart);
void USART_SendData(USART_TypeDef* USARTx, uint16_t Data);
#endif

