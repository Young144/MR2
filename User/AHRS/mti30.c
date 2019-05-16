/**
*	接受mti30的函数 采用一个任务 DMA接收
* 注意mti30数据 高低位是反的 需要先翻转过来 （该过程在bsp_can里面已经完成）
  */


#include "mti30.h"

//预留 ******
/*目前的代码在bsp_can里面 
*如果代码多的话
*考虑移植问题 
*因该将之放到这里写
*/

/**************************************一维卡尔曼滤波器**************************************/
/*       
        Q:过程噪声，Q增大，动态响应变快，收敛稳定性变坏
        R:测量噪声，R增大，动态响应变慢，收敛稳定性变好       
*/

/* 卡尔曼滤波处理 */

float KalmanFilter( float ResrcData,float ProcessNiose_Q,float MeasureNoise_R)
{

    float R = MeasureNoise_R;
    float Q = ProcessNiose_Q;

    static float x_last;
    float x_mid = x_last;
    float x_now;

    static float p_last;
    float p_mid ;
    float p_now;

    float kg;

    x_mid=x_last;                       //x_last=x(k-1|k-1),x_mid=x(k|k-1)
    p_mid=p_last+Q;                     //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声

    /*
     *  卡尔曼滤波的五个重要公式
     */
    kg=p_mid/(p_mid+R);                 //kg为kalman filter，R 为噪声
    x_now=x_mid+kg*(ResrcData-x_mid);   //估计出的最优值
    p_now=(1-kg)*p_mid;                 //最优值对应的covariance
    p_last = p_now;                     //更新covariance 值
    x_last = x_now;                     //更新系统状态值

    return x_now;

}

