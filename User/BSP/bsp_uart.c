/**
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 * @file       bsp_uart.c
 * @brief      this file contains rc data receive and processing function
 * @note
 * @Version    V1.0.0
 * @Date       Jan-30-2018
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 */


/*
*****移植了DJI的中断逻辑处理函数
*****使用串口空闲中断来接收数据比较合理
*/

#define ABS(x)		((x>0)? (x): (-x))

#include "bsp_uart.h"

uint8_t   imu_buf[IMU_BUFLEN];

bool imu_rec_flag;

uint8_t   openmv_buf[OPENMV_BUFLEN];

uint8_t   usart3_buf[USART3_BUFLEN];

uint8_t   usart2_buf[USART2_BUFLEN];

imudata imuinfo;

float imu_filter_data[10];

float yaw_calibrated;

uint8_t _count_imu=0;

openmvdata openmvinfo;

ps2data ps2info;

float _Pitch_initial,_Roll_initial,_Pitch_rev,_Roll_rev;

/**
  * @brief      enable global uart it and do not use DMA transfer done it
  * @param[in]  huart: uart IRQHandler id
  * @param[in]  pData: receive buff
  * @param[in]  Size:  buff size
  * @retval     set success or fail
  */
static int uart_receive_dma_no_it(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size)
{
    uint32_t tmp1 = 0;

    tmp1 = huart->RxState;

    if (tmp1 == HAL_UART_STATE_READY)
    {
        if ((pData == NULL) || (Size == 0))
        {
            return HAL_ERROR;
        }

        huart->pRxBuffPtr = pData;
        huart->RxXferSize = Size;
        huart->ErrorCode  = HAL_UART_ERROR_NONE;

        /* Enable the DMA Stream */
        HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, (uint32_t)pData, Size);

        /*
         * Enable the DMA transfer for the receiver request by setting the DMAR bit
         * in the UART CR3 register
         */
        SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

/**
  * @brief      returns the number of remaining data units in the current DMAy Streamx transfer.
  * @param[in]  dma_stream: where y can be 1 or 2 to select the DMA and x can be 0
  *             to 7 to select the DMA Stream.
  * @retval     The number of remaining data units in the current DMAy Streamx transfer.
  */
uint16_t dma_current_data_counter(DMA_Stream_TypeDef *dma_stream)
{
    /* Return the number of remaining data units for DMAy Streamx */
    return ((uint16_t)(dma_stream->NDTR));
}

/************************************空闲中断回调函数功能函数区************************************/
void usart2_callback_handler( openmvdata *openmvdata, uint8_t *buff)
{

}

void usart3_callback_handler( ps2data *ps2data, uint8_t *buff)
{

    uint8_t datasum=0;
    for(int i=2; i<8; i++)		//计算十二位数据位和一位校验位的和
        datasum+=buff[i];

    if(datasum==0xff) 		//校验和算法 ：取最后一个字节如果为ff 检验通过
    {
        ps2data->KEY_VALUE = buff[2];
        ps2data->PSS_LX_VALUE = buff[3];
        ps2data->PSS_LY_VALUE = buff[4];
        ps2data->PSS_RX_VALUE = buff[5];
        ps2data->PSS_RY_VALUE = buff[6];

    }

}

void imu_callback_handler( imudata *imudata, uint8_t *buff)
{
	
	
//    if(buff[0]==0xfa&&buff[1]==0xff  mti30的数据解析部分
//    {
//        uint8_t datasum=0;
//        for(int i=7; i<20; i++)		//计算十二位数据位和一位校验位的和
//            datasum+=buff[i];

//        if((datasum&0x00f)==0x000) 		//取最后一个字节如果为00 检验通过
//        {
//            for(int i = 0; i<12; i++)
//                imudata->data[i] = buff[18-i];  //mti30的数据高字节在低地址 所以反向存储 ActVal 0 1 2分别为 YAW PITCH ROll
//        }
//    }

    if(buff[0]==0x0D&&buff[1]==0x0A)
    {
			imu_rec_flag=1;
        for(int i = 0; i<12; i++)
            imudata->data[i] = buff[i+2];
    }
		
    _count_imu++;
    imu_filter_data[_count_imu]=imudata->ActVal[0];

    if(_count_imu==9) {
        _count_imu=0;

        for( int i=0; i<10 ; i++)
            yaw_calibrated+=imu_filter_data[i];

        yaw_calibrated/=10;

    }


}

void openmv_callback_handler( openmvdata *openmvdata, uint8_t *buff)
{
    if(buff[0]==0xff&&buff[1]==0xfa)
    {
        for(int i = 0; i<12; i++)
            openmvdata->data[i] = buff[i+2];
    }
}
/************************************END************************************/



/**
  * @brief      clear idle it flag after uart receive a frame data
  * @param[in]  huart: uart IRQHandler id
  * @retval
  */
static void uart_rx_idle_callback(UART_HandleTypeDef* huart)
{
    /* clear idle it flag avoid idle interrupt all the time */
    __HAL_UART_CLEAR_IDLEFLAG(huart);

    if (huart == &USART2_HUART)
    {
        __HAL_DMA_DISABLE(huart->hdmarx);

        if ((USART2_MAX_LEN - dma_current_data_counter(huart->hdmarx->Instance)) == USART2_BUFLEN)
            usart2_callback_handler(&openmvinfo, usart2_buf);

        /* restart dma transmission */
        __HAL_DMA_SET_COUNTER(huart->hdmarx, IMU_MAX_LEN);
        __HAL_DMA_ENABLE(huart->hdmarx);

    }
    else if (huart == &USART3_HUART)
    {
        __HAL_DMA_DISABLE(huart->hdmarx);

        if ((USART3_MAX_LEN - dma_current_data_counter(huart->hdmarx->Instance)) == USART3_BUFLEN)
            usart3_callback_handler(&ps2info, usart3_buf);

        /* restart dma transmission */
        __HAL_DMA_SET_COUNTER(huart->hdmarx, IMU_MAX_LEN);
        __HAL_DMA_ENABLE(huart->hdmarx);

    }
    /* handle received data in idle interrupt */
    else if (huart == &IMU_HUART)
    {
        /* clear DMA transfer complete flag */
        __HAL_DMA_DISABLE(huart->hdmarx);

        if ((IMU_MAX_LEN - dma_current_data_counter(huart->hdmarx->Instance)) == IMU_BUFLEN)
            imu_callback_handler(&imuinfo, imu_buf);

        /* restart dma transmission */
        __HAL_DMA_SET_COUNTER(huart->hdmarx, IMU_MAX_LEN);
        __HAL_DMA_ENABLE(huart->hdmarx);

    }
    else if (huart == &OPENMV_HUART)
    {
        __HAL_DMA_DISABLE(huart->hdmarx);

        if ((OPENMV_MAX_LEN - dma_current_data_counter(huart->hdmarx->Instance)) == OPENMV_BUFLEN)
            openmv_callback_handler(&openmvinfo, openmv_buf);

        /* restart dma transmission */
        __HAL_DMA_SET_COUNTER(huart->hdmarx, IMU_MAX_LEN);
        __HAL_DMA_ENABLE(huart->hdmarx);

    }


}


/**
  * @brief      callback this function when uart interrupt
  * @param[in]  huart: uart IRQHandler id
  * @retval
  */
void uart_receive_handler(UART_HandleTypeDef *huart)
{
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) &&
            __HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
    {
        uart_rx_idle_callback(huart);
    }

}

/**
**空闲中断初始化函数，main里面调用，选择串口即可初始化
**/
void uart_receive_init(UART_HandleTypeDef *huart)
{
    if (huart == &USART2_HUART)
    {
        __HAL_UART_CLEAR_IDLEFLAG(&USART2_HUART);
        __HAL_UART_ENABLE_IT(&USART2_HUART, UART_IT_IDLE);
        uart_receive_dma_no_it(&USART2_HUART, usart2_buf, USART2_MAX_LEN);
    }
    else if (huart == &USART3_HUART)
    {
        __HAL_UART_CLEAR_IDLEFLAG(&USART3_HUART);
        __HAL_UART_ENABLE_IT(&USART3_HUART, UART_IT_IDLE);
        uart_receive_dma_no_it(&USART3_HUART, usart3_buf, USART3_MAX_LEN);
    }
    else if (huart == &IMU_HUART)
    {
        __HAL_UART_CLEAR_IDLEFLAG(&IMU_HUART);
        __HAL_UART_ENABLE_IT(&IMU_HUART, UART_IT_IDLE);
        uart_receive_dma_no_it(&IMU_HUART, imu_buf, IMU_MAX_LEN);
    }
    else if (huart == &OPENMV_HUART)
    {
        __HAL_UART_CLEAR_IDLEFLAG(&OPENMV_HUART);
        __HAL_UART_ENABLE_IT(&OPENMV_HUART, UART_IT_IDLE);
        uart_receive_dma_no_it(&OPENMV_HUART, openmv_buf, OPENMV_MAX_LEN);
    }


}


//void ps2_mix( moto *ps2_value ,ps2data *data,float SPEED_MAX , float W_MAX)
//{
//    int LX,LY,RX,RY;
//		float CALC_LX,CALC_LY,CALC_RX,CALC_RY;
//		float value,angle,angle_w;
//
//    LX=data->PSS_LX_VALUE-128;
//    LY=data->PSS_LY_VALUE-128;
//    RX=data->PSS_RX_VALUE-128;
//    RY=data->PSS_RY_VALUE-128;

//    CALC_LX=ABS(LX)*SPEED_MAX/128;
//    CALC_LY=ABS(LY)*SPEED_MAX/128;
//    CALC_RX=ABS(RX)*W_MAX/128;
//    CALC_RY=ABS(RY)*W_MAX/128;

//    value=(float)sqrt(pow(CALC_LX,2)+pow(CALC_LY,2));
//    angle=(float)atan(ABS(CALC_LY)/ABS(CALC_LX));
//    angle_w=(float)CALC_RX;


//    if(LX>0&&LY<0)//1
//			;
//		else if(LX<0&&LY<0)//2
//				angle=180-angle;
//		else if(LX<0&&LY>0)//3
//				angle=-(180-angle);
//		else if(LX>0&&LY>0)//4
//				angle=-angle;

//    if(CALC_RX>0)//
//			;
//		else if(CALC_RX<0)//
//				angle_w=-angle;

//ps2_t->value=value;
//ps2_t->angle=angle;
//ps2_t->angle_w=angle_w;
//}