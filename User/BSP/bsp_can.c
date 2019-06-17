/******************************************************************************
/// @brief
/// @copyright Copyright (c) 2017 <dji-innovations, Corp. RM Dept.>
/// @license MIT License
/// Permission is hereby granted, free of charge, to any person obtaining a copy
/// of this software and associated documentation files (the "Software"), to deal
/// in the Software without restriction,including without limitation the rights
/// to use, copy, modify, merge, publish, distribute, sublicense,and/or sell
/// copies of the Software, and to permit persons to whom the Software is furnished
/// to do so,subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in
/// all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
/// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
/// THE SOFTWARE.
*******************************************************************************/

#include "can.h"
#include "bsp_can.h"
#include "cmsis_os.h"


//moto_measure_t moto_pit;
//moto_measure_t moto_yaw;
//moto_measure_t moto_poke;	//拨单电机
moto_measure_t moto_chassis[8] = {0};
moto_measure_t moto_info;


void get_total_angle(moto_measure_t *p);
void get_moto_offset(moto_measure_t *ptr, CAN_HandleTypeDef* hcan);

/*******************************************************************************************
  * @Func		my_can_filter_init
  * @Brief    CAN1和CAN2滤波器配置
  * @Param		CAN_HandleTypeDef* hcan
  * @Retval		None
  * @Date     2015/11/30
 *******************************************************************************************/
void my_can_filter_init_recv_all(CAN_HandleTypeDef* _hcan)
{
    //can1 &can2 use same filter config
    CAN_FilterConfTypeDef		CAN_FilterConfigStructure;
    static CanTxMsgTypeDef		Tx1Message;
    static CanRxMsgTypeDef 		Rx1Message;
    static CanTxMsgTypeDef		Tx2Message;
    static CanRxMsgTypeDef 		Rx2Message;

    CAN_FilterConfigStructure.FilterNumber = 0;
    CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_FilterConfigStructure.FilterIdHigh = 0x0000;
    CAN_FilterConfigStructure.FilterIdLow = 0x0000;
    CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;
    CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;
    CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0;
    CAN_FilterConfigStructure.BankNumber = 14;//can1(0-13)和can2(14-27)分别得到一半的filter
    CAN_FilterConfigStructure.FilterActivation = ENABLE;

    if(HAL_CAN_ConfigFilter(_hcan, &CAN_FilterConfigStructure) != HAL_OK)
    {
        //err_deadloop(); //show error!
    }

    //filter config for can2
    //can1(0-13)和can2(14-27)分别得到一半的filter
    CAN_FilterConfigStructure.FilterNumber = 14;
    if(HAL_CAN_ConfigFilter(_hcan, &CAN_FilterConfigStructure) != HAL_OK)
    {
        //err_deadloop();
    }

    if(_hcan == &hcan1) {
        _hcan->pTxMsg = &Tx1Message;
        _hcan->pRxMsg = &Rx1Message;
    }

    if(_hcan == &hcan2) {
        _hcan->pTxMsg = &Tx2Message;
        _hcan->pRxMsg = &Rx2Message;
    }

}

/*******************************************************************************************
  * @Func			void can_filter_recv_special(CAN_HandleTypeDef* _hcan, s16 id)
  * @Brief    待测试！！！
  * @Param		只接收filtered id，其他的全屏蔽。
  * @Retval		eg： 	CAN1_FilterConfiguration(0, HOST_CONTROL_ID);
										CAN1_FilterConfiguration(1, SET_CURRENT_ID);
										CAN1_FilterConfiguration(2, SET_VOLTAGE_ID);
										CAN1_FilterConfiguration(3, ESC_CAN_DEVICE_ID);
										CAN1_FilterConfiguration(4, SET_POWER_ID);
										CAN1_FilterConfiguration(5, SET_LIMIT_RECOVER_ID);
  * @Date     2016年11月11日
 *******************************************************************************************/
void can_filter_recv_special(CAN_HandleTypeDef* hcan, uint8_t filter_number, uint16_t filtered_id)
{
    CAN_FilterConfTypeDef   cf;
    cf.FilterNumber = filter_number;	//过滤器组编号
    cf.FilterMode = CAN_FILTERMODE_IDMASK;	//id屏蔽模式
    cf.FilterScale = CAN_FILTERSCALE_32BIT;	//32bit 滤波
    cf.FilterIdHigh = (filtered_id<<21) >> 16;	//high 16 bit		其实这两个结构体成员变量是16位宽
    cf.FilterIdLow = filtered_id<<21;	//low 16bit
    cf.FilterMaskIdHigh = 0xFFFF;
    cf.FilterMaskIdLow = 0xFFF8;	//IDE[2], RTR[1] TXRQ[0] 低三位不考虑。
    cf.FilterFIFOAssignment = CAN_FilterFIFO0;
    cf.BankNumber = 14;	//can1(0-13)和can2(14-27)分别得到一半的filter
    cf.FilterActivation = ENABLE;
    HAL_CAN_ConfigFilter(hcan, &cf);
}


HAL_StatusTypeDef can_send_msg()
{
//	if(_hcan->Instance->ESR){
//		//can error occured, sleep can and reset!
//		_hcan->Instance->MCR |= 0x02;
//		_hcan->Instance->MCR &= ~(0x02);
//	}//这个是zw试过的可以解决can错误  有待验证！
    return HAL_OK;
}

float ZGyroModuleAngle;
/*******************************************************************************************
  * @Func			void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan)
  * @Brief    这是一个回调函数,都不用声明
  * @Param
  * @Retval		None
  * @Date     2015/11/24
 *******************************************************************************************/
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan)
{
    //ignore can1 or can2.
    switch(_hcan->pRxMsg->StdId) {
    case CAN_3508Moto1_ID:
    case CAN_3508Moto2_ID:
    case CAN_3508Moto3_ID:
    case CAN_3508Moto4_ID:
    case CAN_3508Moto5_ID:
    case CAN_3508Moto6_ID:
    case CAN_3508Moto7_ID:
    case CAN_3508Moto8_ID:
    {
        static u8 i;
        i = _hcan->pRxMsg->StdId - CAN_3508Moto1_ID;

        moto_chassis[i].msg_cnt++ <= 50	?	get_moto_offset(&moto_chassis[i], _hcan) : get_moto_measure(&moto_chassis[i], _hcan);
        get_moto_measure(&moto_info, _hcan);
        //get_moto_measure(&moto_chassis[i], _hcan);
    }
    break;


    }


    //hcan1.Instance->IER|=0x00008F02;
    /*#### add enable can it again to solve can receive only one ID problem!!!####**/
    __HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_FMP0);
    __HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_FMP0);
    /*#### add enable can it again to solve can receive only one ID problem!!!####**/
}

/*******************************************************************************************
  * @Func			void get_moto_measure(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
  * @Brief    接收云台电机,3510电机通过CAN发过来的信息
  * @Param
  * @Retval		None
  * @Date     2015/11/24
 *******************************************************************************************/
void get_moto_measure(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
{
//	u32  sum=0;
//	u8	 i = FILTER_BUF_LEN;

    /*BUG!!! dont use this para code*/
//	ptr->angle_buf[ptr->buf_idx] = (uint16_t)(hcan->pRxMsg->Data[0]<<8 | hcan->pRxMsg->Data[1]) ;
//	ptr->buf_idx = ptr->buf_idx++ > FILTER_BUF_LEN ? 0 : ptr->buf_idx;
//	while(i){
//		sum += ptr->angle_buf[--i];
//	}
//	ptr->fited_angle = sum / FILTER_BUF_LEN;
    ptr->last_angle = ptr->angle;
    ptr->angle = (uint16_t)(hcan->pRxMsg->Data[0]<<8 | hcan->pRxMsg->Data[1]) ;
    ptr->speed_rpm = (int16_t)(hcan->pRxMsg->Data[2]<<8 | hcan->pRxMsg->Data[3]);

    //ptr->real_current  = (int16_t)(hcan->pRxMsg->Data[2]<<8 | hcan->pRxMsg->Data[3]);
    //ptr->speed_rpm = ptr->real_current;	//这里是因为两种电调对应位不一样的信息
    //ptr->given_current = (int16_t)(hcan->pRxMsg->Data[4]<<8 | hcan->pRxMsg->Data[5]);///-5;
    //ptr->given_current = (int16_t)(hcan->pRxMsg->Data[4]<<8 | hcan->pRxMsg->Data[5]) ;//获取电流(mm/s) / 819.2

    ptr->real_current = ((int16_t)(hcan->pRxMsg->Data[4]<<8 | hcan->pRxMsg->Data[5])) / 819.2;//获取电流(mm/s) / 819.2
    //(ptr)->given_current = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]);
    //ptr->given_current = (hcan->pRxMsg->Data[4]<<8 | hcan->pRxMsg->Data[5])*5.f/16384.f;//获得转矩
    ptr->hall = hcan->pRxMsg->Data[6];
    if(ptr->angle - ptr->last_angle > 4096)
        ptr->round_cnt --;
    else if (ptr->angle - ptr->last_angle < -4096)
        ptr->round_cnt ++;
    ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
}

/*this function should be called after system+can init */
void get_moto_offset(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
{
    ptr->angle = (uint16_t)(hcan->pRxMsg->Data[0]<<8 | hcan->pRxMsg->Data[1]) ;
    ptr->offset_angle = ptr->angle;
}

#define ABS(x)	( (x>0) ? (x) : (-x) )
/**
*@bref 电机上电角度=0， 之后用这个函数更新3510电机的相对开机后（为0）的相对角度。
	*/
void get_total_angle(moto_measure_t *p) {

    int res1, res2, delta;
    if(p->angle < p->last_angle) {			//可能的情况
        res1 = p->angle + 8192 - p->last_angle;	//正转，delta=+
        res2 = p->angle - p->last_angle;				//反转	delta=-
    } else {	//angle > last
        res1 = p->angle - 8192 - p->last_angle ;//反转	delta -
        res2 = p->angle - p->last_angle;				//正转	delta +
    }
    //不管正反转，肯定是转的角度小的那个是真的
    if(ABS(res1)<ABS(res2))
        delta = res1;
    else
        delta = res2;

    p->total_angle += delta;
    p->last_angle = p->angle;
}

//void set_moto_current(CAN_HandleTypeDef* hcan, s16 iq1, s16 iq2, s16 iq3, s16 iq4){

//	hcan->pTxMsg->StdId = 0x200;
//	hcan->pTxMsg->IDE = CAN_ID_STD;
//	hcan->pTxMsg->RTR = CAN_RTR_DATA;
//	hcan->pTxMsg->DLC = 0x08;
//	hcan->pTxMsg->Data[0] = iq1 >> 8;
//	hcan->pTxMsg->Data[1] = iq1;
//	hcan->pTxMsg->Data[2] = iq2 >> 8;
//	hcan->pTxMsg->Data[3] = iq2;
//	hcan->pTxMsg->Data[4] = iq3 >> 8;
//	hcan->pTxMsg->Data[5] = iq3;
//	hcan->pTxMsg->Data[6] = iq4 >> 8;
//	hcan->pTxMsg->Data[7] = iq4;
//
//	HAL_CAN_Transmit(hcan, 1000);
//}
