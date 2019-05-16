/**
  ******************************************************************************
  * @file			vcan.c
  * @version		V1.0.0
  * @date			4.17 ylt
  * @brief	开启了8个通道 使用山外串口调试助手打印波形 可以自己添加通道 方便调试
  *******************************************************************************/

//*山外地面站   vcan ground control
#include "vcan.h"

uint8_t VcanTxBuff[20]= {0};
short  wave_form_data[8] = {0};

void vcan_send_byte(uint8_t date);
void Vcan_Send_Wave_Data(void);


void VcanGC_task(void *pvParameters)
{
    extern int count;
    for(;;)
    {
        wave_form_data[0] =(char)imuinfo.ActVal[2];
        wave_form_data[1] =(char)imuinfo.ActVal[1];
        wave_form_data[2] =(char)imuinfo.ActVal[0];
        wave_form_data[3] =10;
//        wave_form_data[4] =moto_chassis[4].total_angle/ReductionAndAngleRatio;
//        wave_form_data[5] =moto_chassis[5].total_angle/ReductionAndAngleRatio;
//        wave_form_data[6] =moto_chassis[6].total_angle/ReductionAndAngleRatio;
//        wave_form_data[7] =moto_chassis[7].total_angle/ReductionAndAngleRatio;

        Vcan_Send_Wave_Data();
        vTaskDelay(200);
    }

}

void Vcan_Send_Wave_Data(void)
{
    VcanTxBuff[0]=0x03;
    VcanTxBuff[1]=0xfc;

//    VcanTxBuff[2]=(wave_form_data[0]&0xff);
//    VcanTxBuff[3]=(wave_form_data[0]>>8);
//    VcanTxBuff[4]=(wave_form_data[1]&0xff);
//    VcanTxBuff[5]=(wave_form_data[1]>>8);
//    VcanTxBuff[6]=(wave_form_data[2]&0xff);
//    VcanTxBuff[7]=(wave_form_data[2]>>8);
//    VcanTxBuff[8]=(wave_form_data[3]&0xff);
//    VcanTxBuff[9]=(wave_form_data[3]>>8);
//    VcanTxBuff[10]=(wave_form_data[4]&0xff);
//    VcanTxBuff[11]=(wave_form_data[4]>>8);
//    VcanTxBuff[12]=(wave_form_data[5]&0xff);
//    VcanTxBuff[13]=(wave_form_data[5]>>8);
//    VcanTxBuff[14]=(wave_form_data[6]&0xff);
//    VcanTxBuff[15]=(wave_form_data[6]>>8);
//    VcanTxBuff[16]=(wave_form_data[7]&0xff);
//    VcanTxBuff[17]=(wave_form_data[7]>>8);
	
for(int i =0;i<12;i++)
  VcanTxBuff[i+2]=imuinfo.data[11-i];


    VcanTxBuff[14]=0xfc;
    VcanTxBuff[15]=0x03;


    HAL_UART_Transmit_DMA(&huart8, (uint8_t*)&VcanTxBuff, sizeof(VcanTxBuff));

}
