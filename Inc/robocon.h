/**
  ******************************************************************************
  * @file           : __ROBOCON_H.h
  * @brief          : 所有的用户添加的比赛函数头文件都放到这儿
  *                   使用时只需要包含此文件即可
  ******************************************************************************
  ******************************************************************************
  */

#ifndef __ROBOCON_H
#define __ROBOCON_H



/* 系统 ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "main.h"

#include "string.h"
#include "stdlib.h"

/* 外设 ------------------------------------------------------------------*/
#include "mytype.h"
#include "usart.h"
#include "gpio.h"
#include "math.h"
#include "can.h"
#include "tim.h"
#include "spi.h"
#include "dma.h"

/* APP ------------------------------------------------------------------*/
#include "combinations.h"
#include "power_ctrl.h"
#include "jump.h"

/* TASK ------------------------------------------------------------------*/
#include "posture_ctrl.h"
#include "moto_ctrl.h"
#include "debug.h"
#include "navigation.h"
#include "Remote_Control.h"
#include "logicalflow_task.h"
#include "detect_task.h"

/* AHRS ------------------------------------------------------------------*/
#include "imu.h"
#include "mti30.h"

/* HARDWARE ------------------------------------------------------------------*/
#include "buzzer.h"
#include "led.h"
#include "servo.h"
#include "stepmotor.h"


/* BSP ------------------------------------------------------------------*/
#include "bsp_uart.h"
#include "bsp_can.h"
#include "pid.h"
#include "vcan.h"
#include "robomoudle.h"

#endif /* __ROBOCON_H */
