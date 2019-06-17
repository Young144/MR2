
#include "robomoudle.h"

unsigned char can_tx_success_flag = 0;
unsigned int CAN_Time_Out = 0;

void robomoudle_init(void)
{
//    vTaskDelay(500);
//    CAN_RoboModule_DRV_Reset(1,1);
//    vTaskDelay(500);
//    CAN_RoboModule_DRV_Config(1,1,0,0);
    //vTaskDelay(1000);
    IndLED_On(IndColorBlue);

    CAN_RoboModule_DRV_Reset(0,1);
    vTaskDelay(100);
	
    IndLED_Off();
	
    CAN_RoboModule_DRV_Config(0,1,100,0);
    vTaskDelay(100);
	
    IndLED_On(IndColorBlue);
	
//    CAN_RoboModule_DRV_Mode_Choice(1,1,OpenLoop_Mode);
//    vTaskDelay(500);
//    CAN_RoboModule_DRV_OpenLoop_Mode(1,1,0);
//    vTaskDelay(500);

    CAN_RoboModule_DRV_Mode_Choice(0,1,Position_Mode);
    vTaskDelay(100);
		
		 IndLED_Off();
		 
    CAN_RoboModule_DRV_Position_Mode( 0, 1, 500, 0);
    vTaskDelay(100);
		
		IndLED_On(IndColorBlue);
		
		vTaskDelay(200);
		
		IndLED_Off();
		
		



//
//	GPIO_InitTypeDef GPIO_Initure;          //??????????
//
//	   __HAL_RCC_GPIOH_CLK_ENABLE();           //????GPIOH???
//
//		GPIO_Initure.Pin=GPIO_PIN_10;            //PH10   ??????????
//    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;   //????
//    GPIO_Initure.Pull=GPIO_PULLUP;        //????
//    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //????
//    HAL_GPIO_Init(GPIOH,&GPIO_Initure);
//
//	  GPIO_Initure.Pin=GPIO_PIN_11;            //PH11   ?????????
//    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;   //????
//    GPIO_Initure.Pull=GPIO_PULLUP;        //????
//    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //????
//    HAL_GPIO_Init(GPIOH,&GPIO_Initure);
//
//		GPIO_Initure.Pin=GPIO_PIN_12;            //PH12    ??¦Ë????
//    GPIO_Initure.Mode=GPIO_MODE_INPUT;     //????
//    GPIO_Initure.Pull=GPIO_PULLUP;        //????
//    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //????
//    HAL_GPIO_Init(GPIOH,&GPIO_Initure);
//
//		HAL_GPIO_WritePin(GPIOH,GPIO_PIN_11,GPIO_PIN_SET);	//PH11??0
//    HAL_GPIO_WritePin(GPIOH,GPIO_PIN_10,GPIO_PIN_SET);	//PH11??0
}

static void CAN_Delay_Us(unsigned int t)
{
    int i;
    for(i=0; i<t; i++)
    {
        int a=40;
        while(a--);
    }
}

/****************************************************************************************
                                       ??¦Ë???
Group   ????¦¶ 0-7
Number  ????¦¶ 0-15??????Number==0??????????
*****************************************************************************************/
void CAN_RoboModule_DRV_Reset(unsigned char Group,unsigned char Number)
{
    unsigned short can_id = 0x000;

    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }

    can_tx_success_flag = 0;

    hcan1.pTxMsg->StdId = can_id;
    hcan1.pTxMsg->IDE = CAN_ID_STD;
    hcan1.pTxMsg->RTR = CAN_RTR_DATA;
    hcan1.pTxMsg->DLC = 0x08;
    hcan1.pTxMsg->Data[0] = 0x55;
    hcan1.pTxMsg->Data[1] = 0x55;
    hcan1.pTxMsg->Data[2] = 0x55;
    hcan1.pTxMsg->Data[3] = 0x55;
    hcan1.pTxMsg->Data[4] = 0x55;
    hcan1.pTxMsg->Data[5] = 0x55;
    hcan1.pTxMsg->Data[6] = 0x55;
    hcan1.pTxMsg->Data[7] = 0x55;

    HAL_CAN_Transmit(&hcan1, 1000);

    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}

/****************************************************************************************
                                     ????????
Group   ????¦¶ 0-7
Number  ????¦¶ 0-15??????Number==0??????????

Mode    ????¦¶

OpenLoop_Mode                       0x01
Current_Mode                        0x02
Velocity_Mode                       0x03
Position_Mode                       0x04
Velocity_Position_Mode              0x05
Current_Velocity_Mode               0x06
Current_Position_Mode               0x07
Current_Velocity_Position_Mode      0x08
*****************************************************************************************/
void CAN_RoboModule_DRV_Mode_Choice(unsigned char Group,unsigned char Number,unsigned char Mode)
{
    unsigned short can_id = 0x001;

    hcan1.pTxMsg->IDE = CAN_ID_STD;
    hcan1.pTxMsg->RTR = CAN_RTR_DATA;
    hcan1.pTxMsg->DLC = 0x08;

    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }

    hcan1.pTxMsg->StdId = can_id;     //?ID??????????CAN_ID

    hcan1.pTxMsg->Data[0] = Mode;
    hcan1.pTxMsg->Data[1] = 0x55;
    hcan1.pTxMsg->Data[2] = 0x55;
    hcan1.pTxMsg->Data[3] = 0x55;
    hcan1.pTxMsg->Data[4] = 0x55;
    hcan1.pTxMsg->Data[5] = 0x55;
    hcan1.pTxMsg->Data[6] = 0x55;
    hcan1.pTxMsg->Data[7] = 0x55;


    can_tx_success_flag = 0;
    HAL_CAN_Transmit(&hcan1, 1000);
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}

/****************************************************************************************
                                   ????????????????
Group   ????¦¶ 0-7

Number  ????¦¶ 0-15??????Number==0??????????

temp_pwm??????¦¶?????
0 ~ +5000?????5000??????temp_pwm = ??5000???????????????????

*****************************************************************************************/
void CAN_RoboModule_DRV_OpenLoop_Mode(unsigned char Group,unsigned char Number,short Temp_PWM)
{
    unsigned short can_id = 0x002;

    hcan1.pTxMsg->IDE = CAN_ID_STD;
    hcan1.pTxMsg->RTR = CAN_RTR_DATA;
    hcan1.pTxMsg->DLC = 0x08;

    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }

    hcan1.pTxMsg->StdId = can_id;     //?ID??????????CAN_ID


    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }

    hcan1.pTxMsg->Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    hcan1.pTxMsg->Data[1] = (unsigned char)(Temp_PWM&0xff);
    hcan1.pTxMsg->Data[2] = 0x55;
    hcan1.pTxMsg->Data[3] = 0x55;
    hcan1.pTxMsg->Data[4] = 0x55;
    hcan1.pTxMsg->Data[5] = 0x55;
    hcan1.pTxMsg->Data[6] = 0x55;
    hcan1.pTxMsg->Data[7] = 0x55;

    can_tx_success_flag = 0;
    HAL_CAN_Transmit(&hcan1, 1000);

    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}

/****************************************************************************************
                                   ????????????????
Group   ????¦¶ 0-7

Number  ????¦¶ 0-15??????Number==0??????????

temp_pwm??????¦¶?????
0 ~ +5000?????5000??????temp_pwm = 5000???????????????????

temp_current??????¦¶?????
-32768 ~ +32767????¦ËmA

*****************************************************************************************/
//void CAN_RoboModule_DRV_Current_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Current)
//{
//    unsigned short can_id = 0x003;
//    CanTxMsgTypeDef tx_message;

//    tx_message.IDE = CAN_ID_STD;    //????
//    tx_message.RTR = CAN_RTR_DATA;  //?????
//    tx_message.DLC = 0x08;          //??????8

//    if((Group<=7)&&(Number<=15))
//    {
//        can_id |= Group<<8;
//        can_id |= Number<<4;
//    }
//    else
//    {
//        return;
//    }

//    tx_message.StdId = can_id;      //?ID??????????CAN_ID

//    if(Temp_PWM > 5000)
//    {
//        Temp_PWM = 5000;
//    }
//    else if(Temp_PWM < -5000)
//    {
//        Temp_PWM = -5000;
//    }

//    if(Temp_PWM < 0)
//    {
//        Temp_PWM = abs(Temp_PWM);
//    }

//    tx_message.Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
//    tx_message.Data[1] = (unsigned char)(Temp_PWM&0xff);
//    tx_message.Data[2] = (unsigned char)((Temp_Current>>8)&0xff);
//    tx_message.Data[3] = (unsigned char)(Temp_Current&0xff);
//    tx_message.Data[4] = 0x55;
//    tx_message.Data[5] = 0x55;
//    tx_message.Data[6] = 0x55;
//    tx_message.Data[7] = 0x55;

//    can_tx_success_flag = 0;
//    HAL_CAN_Transmit(&hcan1, 1000);

//    CAN_Time_Out = 0;
//    while(can_tx_success_flag == 0)
//    {
//        CAN_Delay_Us(1);
//        CAN_Time_Out++;
//        if(CAN_Time_Out>100)
//        {
//            break;
//        }
//    }
//}

/****************************************************************************************
                                   ???????????????
Group   ????¦¶ 0-7

Number  ????¦¶ 0-15??????Number==0??????????

temp_pwm??????¦¶?????
0 ~ +5000?????5000??????temp_pwm = 5000???????????????????

temp_velocity??????¦¶?????
-32768 ~ +32767????¦ËRPM

*****************************************************************************************/
void CAN_RoboModule_DRV_Velocity_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Velocity)
{
    unsigned short can_id = 0x004;
    CanTxMsgTypeDef tx_message;

    tx_message.IDE = CAN_ID_STD;    //????
    tx_message.RTR = CAN_RTR_DATA;  //?????
    tx_message.DLC = 0x08;          //??????8

    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }

    tx_message.StdId = can_id;      //?ID??????????CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }

    if(Temp_PWM < 0)
    {
        Temp_PWM = abs(Temp_PWM);
    }

    tx_message.Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_PWM&0xff);
    tx_message.Data[2] = (unsigned char)((Temp_Velocity>>8)&0xff);
    tx_message.Data[3] = (unsigned char)(Temp_Velocity&0xff);
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;

    can_tx_success_flag = 0;
    HAL_CAN_Transmit(&hcan1, 1000);

    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}

/****************************************************************************************
                                   ¦Ë??????????????
Group   ????¦¶ 0-7

Number  ????¦¶ 0-15??????Number==0??????????

temp_pwm??????¦¶?????
0 ~ +5000?????5000??????temp_pwm = 5000???????????????????

temp_position??????¦¶?????
-2147483648~+2147483647????¦Ëqc

*****************************************************************************************/
void CAN_RoboModule_DRV_Position_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,long Temp_Position)
{
    unsigned short can_id = 0x005;

    hcan1.pTxMsg->IDE = CAN_ID_STD;    //????
    hcan1.pTxMsg->RTR = CAN_RTR_DATA;  //?????
    hcan1.pTxMsg->DLC = 0x08;          //??????8

    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }

    hcan1.pTxMsg->StdId = can_id;      //?ID??????????CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }

    if(Temp_PWM < 0)
    {
        Temp_PWM = abs(Temp_PWM);
    }

    hcan1.pTxMsg->Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    hcan1.pTxMsg->Data[1] = (unsigned char)(Temp_PWM&0xff);
    hcan1.pTxMsg->Data[2] = 0x55;
    hcan1.pTxMsg->Data[3] = 0x55;
    hcan1.pTxMsg->Data[4] = (unsigned char)((Temp_Position>>24)&0xff);
    hcan1.pTxMsg->Data[5] = (unsigned char)((Temp_Position>>16)&0xff);
    hcan1.pTxMsg->Data[6] = (unsigned char)((Temp_Position>>8)&0xff);
    hcan1.pTxMsg->Data[7] = (unsigned char)(Temp_Position&0xff);

    can_tx_success_flag = 0;
    HAL_CAN_Transmit(&hcan1, 1000);

    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}

/****************************************************************************************
                                  ???¦Ë??????????????
Group   ????¦¶ 0-7

Number  ????¦¶ 0-15??????Number==0??????????

temp_pwm??????¦¶?????
0 ~ +5000?????5000??????temp_pwm = 5000???????????????????

temp_velocity??????¦¶?????
0 ~ +32767????¦ËRPM

temp_position??????¦¶?????
-2147483648~+2147483647????¦Ëqc
*****************************************************************************************/
void CAN_RoboModule_DRV_Velocity_Position_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Velocity,long Temp_Position)
{
    unsigned short can_id = 0x006;
    CanTxMsgTypeDef tx_message;

    tx_message.IDE = CAN_ID_STD;    //????
    tx_message.RTR = CAN_RTR_DATA;  //?????
    tx_message.DLC = 0x08;          //??????8

    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }

    tx_message.StdId = can_id;      //?ID??????????CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }

    if(Temp_PWM < 0)
    {
        Temp_PWM = abs(Temp_PWM);
    }

    if(Temp_Velocity < 0)
    {
        Temp_Velocity = abs(Temp_Velocity);
    }

    tx_message.Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_PWM&0xff);
    tx_message.Data[2] = (unsigned char)((Temp_Velocity>>8)&0xff);
    tx_message.Data[3] = (unsigned char)(Temp_Velocity&0xff);
    tx_message.Data[4] = (unsigned char)((Temp_Position>>24)&0xff);
    tx_message.Data[5] = (unsigned char)((Temp_Position>>16)&0xff);
    tx_message.Data[6] = (unsigned char)((Temp_Position>>8)&0xff);
    tx_message.Data[7] = (unsigned char)(Temp_Position&0xff);

    can_tx_success_flag = 0;
    HAL_CAN_Transmit(&hcan1, 1000);

    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }

}


/****************************************************************************************
                                  ???????????????????
Group   ????¦¶ 0-7

Number  ????¦¶ 0-15??????Number==0??????????

temp_current??????¦¶?????
0 ~ +32767????¦ËmA

temp_velocity??????¦¶?????
-32768 ~ +32767????¦ËRPM

*****************************************************************************************/
void CAN_RoboModule_DRV_Current_Velocity_Mode(unsigned char Group,unsigned char Number,short Temp_Current,short Temp_Velocity)
{
    unsigned short can_id = 0x007;
    CanTxMsgTypeDef tx_message;

    tx_message.IDE = CAN_ID_STD;    //????
    tx_message.RTR = CAN_RTR_DATA;  //?????
    tx_message.DLC = 0x08;          //??????8

    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }

    tx_message.StdId = can_id;      //?ID??????????CAN_ID

    if(Temp_Current < 0)
    {
        Temp_Current = abs(Temp_Current);
    }

    tx_message.Data[0] = (unsigned char)((Temp_Current>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_Current&0xff);
    tx_message.Data[2] = (unsigned char)((Temp_Velocity>>8)&0xff);
    tx_message.Data[3] = (unsigned char)(Temp_Velocity&0xff);
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;

    can_tx_success_flag = 0;
    HAL_CAN_Transmit(&hcan1, 1000);
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}


/****************************************************************************************
                                  ????¦Ë??????????????
Group   ????¦¶ 0-7

Number  ????¦¶ 0-15??????Number==0??????????

temp_current??????¦¶?????
0 ~ +32767????¦ËmA

temp_position??????¦¶?????
-2147483648~+2147483647????¦Ëqc

*****************************************************************************************/
void CAN_RoboModule_DRV_Current_Position_Mode(unsigned char Group,unsigned char Number,short Temp_Current,long Temp_Position)
{
    unsigned short can_id = 0x008;
    CanTxMsgTypeDef tx_message;

    tx_message.IDE = CAN_ID_STD;    //????
    tx_message.RTR = CAN_RTR_DATA;  //?????
    tx_message.DLC = 0x08;          //??????8

    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }

    tx_message.StdId = can_id;      //?ID??????????CAN_ID


    if(Temp_Current < 0)
    {
        Temp_Current = abs(Temp_Current);
    }

    tx_message.Data[0] = (unsigned char)((Temp_Current>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_Current&0xff);
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = (unsigned char)((Temp_Position>>24)&0xff);
    tx_message.Data[5] = (unsigned char)((Temp_Position>>16)&0xff);
    tx_message.Data[6] = (unsigned char)((Temp_Position>>8)&0xff);
    tx_message.Data[7] = (unsigned char)(Temp_Position&0xff);

    can_tx_success_flag = 0;
    HAL_CAN_Transmit(&hcan1, 1000);
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}


/****************************************************************************************
                                  ???????¦Ë??????????????
Group   ????¦¶ 0-7

Number  ????¦¶ 0-15??????Number==0??????????

temp_current??????¦¶?????
0 ~ +32767????¦ËmA

temp_velocity??????¦¶?????
0 ~ +32767????¦ËRPM

temp_position??????¦¶?????
-2147483648~+2147483647????¦Ëqc

*****************************************************************************************/
void CAN_RoboModule_DRV_Current_Velocity_Position_Mode(unsigned char Group,unsigned char Number,short Temp_Current,short Temp_Velocity,long Temp_Position)
{
    unsigned short can_id = 0x009;
    CanTxMsgTypeDef tx_message;

    tx_message.IDE = CAN_ID_STD;    //????
    tx_message.RTR = CAN_RTR_DATA;  //?????
    tx_message.DLC = 0x08;          //??????8

    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }

    tx_message.StdId = can_id;      //?ID??????????CAN_ID

    if(Temp_Current < 0)
    {
        Temp_Current = abs(Temp_Current);
    }

    if(Temp_Velocity < 0)
    {
        Temp_Velocity = abs(Temp_Velocity);
    }

    tx_message.Data[0] = (unsigned char)((Temp_Current>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_Current&0xff);
    tx_message.Data[2] = (unsigned char)((Temp_Velocity>>8)&0xff);
    tx_message.Data[3] = (unsigned char)(Temp_Velocity&0xff);
    tx_message.Data[4] = (unsigned char)((Temp_Position>>24)&0xff);
    tx_message.Data[5] = (unsigned char)((Temp_Position>>16)&0xff);
    tx_message.Data[6] = (unsigned char)((Temp_Position>>8)&0xff);
    tx_message.Data[7] = (unsigned char)(Temp_Position&0xff);

    can_tx_success_flag = 0;
    HAL_CAN_Transmit(&hcan1, 1000);
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}

/****************************************************************************************
                                      ???????
Temp_Time1??????¦¶: 0 ~ 255???0?????????????¦Ë?¡Â???????
Temp_Time2??????¦¶: 0 ~ 255???0?????????¦Ë??????????
*****************************************************************************************/
void CAN_RoboModule_DRV_Config(unsigned char Group,unsigned char Number,unsigned char Temp_Time1,unsigned char Temp_Time2)
{
    unsigned short can_id = 0x00A;

    hcan1.pTxMsg->IDE = CAN_ID_STD;    //????
    hcan1.pTxMsg->RTR = CAN_RTR_DATA;  //?????
    hcan1.pTxMsg->DLC = 0x08;          //??????8

    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }



    hcan1.pTxMsg->StdId = can_id;     //?ID??????????CAN_ID

    hcan1.pTxMsg->Data[0] = Temp_Time1;
    hcan1.pTxMsg->Data[1] = Temp_Time2;
    hcan1.pTxMsg->Data[2] = 0x55;
    hcan1.pTxMsg->Data[3] = 0x55;
    hcan1.pTxMsg->Data[4] = 0x55;
    hcan1.pTxMsg->Data[5] = 0x55;
    hcan1.pTxMsg->Data[6] = 0x55;
    hcan1.pTxMsg->Data[7] = 0x55;


    can_tx_success_flag = 0;

    HAL_CAN_Transmit(&hcan1, 1000);

    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}

/****************************************************************************************
                                      ???????
*****************************************************************************************/
void CAN_RoboModule_DRV_Online_Check(unsigned char Group,unsigned char Number)
{
    unsigned short can_id = 0x00F;
    CanTxMsgTypeDef tx_message;

    tx_message.IDE = CAN_ID_STD;    //????
    tx_message.RTR = CAN_RTR_DATA;  //?????
    tx_message.DLC = 0x08;          //??????8

    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }

    tx_message.StdId = can_id;      //?ID??????????CAN_ID

    tx_message.Data[0] = 0x55;
    tx_message.Data[1] = 0x55;
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;

    can_tx_success_flag = 0;
    HAL_CAN_Transmit(&hcan1, 1000);
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}

short Real_Current_Value[4] = {0};
short Real_Velocity_Value[4] = {0};
long Real_Position_Value[4] = {0};
char Real_Online[4] = {0};
char Real_Ctl1_Value[4] = {0};
char Real_Ctl2_Value[4] = {0};

////????????????????????4????????????????0?ï…?????1??2??3??4
///*************************************************************************
//                          CAN1_RX0_IRQHandler
//??????CAN1??????§Ø????
//*************************************************************************/
//void CAN1_RX0_IRQHandler(void)
//{
//    CanRxMsgTypeDef rx_message;
//
//    if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
//	{
//        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
//        CAN_Receive(CAN1, CAN_FIFO0, &rx_message);
//
//        if((rx_message.IDE == CAN_Id_Standard)&&(rx_message.IDE == CAN_RTR_Data)&&(rx_message.DLC == 8)) //?????????????????????8
//        {
//            if(rx_message.StdId == 0x1B)
//            {
//                Real_Current_Value[0] = (rx_message.Data[0]<<8)|(rx_message.Data[1]);
//                Real_Velocity_Value[0] = (rx_message.Data[2]<<8)|(rx_message.Data[3]);
//                Real_Position_Value[0] = ((rx_message.Data[4]<<24)|(rx_message.Data[5]<<16)|(rx_message.Data[6]<<8)|(rx_message.Data[7]));
//            }
//            else if(rx_message.StdId == 0x2B)
//            {
//                Real_Current_Value[1] = (rx_message.Data[0]<<8)|(rx_message.Data[1]);
//                Real_Velocity_Value[1] = (rx_message.Data[2]<<8)|(rx_message.Data[3]);
//                Real_Position_Value[1] = ((rx_message.Data[4]<<24)|(rx_message.Data[5]<<16)|(rx_message.Data[6]<<8)|(rx_message.Data[7]));
//            }
//            else if(rx_message.StdId == 0x3B)
//            {
//                Real_Current_Value[2] = (rx_message.Data[0]<<8)|(rx_message.Data[1]);
//                Real_Velocity_Value[2] = (rx_message.Data[2]<<8)|(rx_message.Data[3]);
//                Real_Position_Value[2] = ((rx_message.Data[4]<<24)|(rx_message.Data[5]<<16)|(rx_message.Data[6]<<8)|(rx_message.Data[7]));
//            }
//            else if(rx_message.StdId == 0x4B)
//            {
//                Real_Current_Value[3] = (rx_message.Data[0]<<8)|(rx_message.Data[1]);
//                Real_Velocity_Value[3] = (rx_message.Data[2]<<8)|(rx_message.Data[3]);
//                Real_Position_Value[3] = ((rx_message.Data[4]<<24)|(rx_message.Data[5]<<16)|(rx_message.Data[6]<<8)|(rx_message.Data[7]));
//            }
//            else if(rx_message.StdId == 0x1F)
//            {
//                Real_Online[0] = 1;
//            }
//            else if(rx_message.StdId == 0x2F)
//            {
//                Real_Online[1] = 1;
//            }
//            else if(rx_message.StdId == 0x3F)
//            {
//                Real_Online[2] = 1;
//            }
//            else if(rx_message.StdId == 0x4F)
//            {
//                Real_Online[3] = 1;
//            }
//            else if(rx_message.StdId == 0x1C)
//            {
//                Real_Ctl1_Value[0] = rx_message.Data[0];
//                Real_Ctl2_Value[0] = rx_message.Data[1];
//            }
//            else if(rx_message.StdId == 0x2C)
//            {
//                Real_Ctl1_Value[1] = rx_message.Data[0];
//                Real_Ctl2_Value[1] = rx_message.Data[1];
//            }
//            else if(rx_message.StdId == 0x3C)
//            {
//                Real_Ctl1_Value[2] = rx_message.Data[0];
//                Real_Ctl2_Value[2] = rx_message.Data[1];
//            }
//            else if(rx_message.StdId == 0x4C)
//            {
//                Real_Ctl1_Value[3] = rx_message.Data[0];
//                Real_Ctl2_Value[3] = rx_message.Data[1];
//            }

//        }
//
//    }
//}
