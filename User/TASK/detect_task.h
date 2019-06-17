
#ifndef DETECT_Task_H
#define DETECT_Task_H
#include "robocon.h"
//错误码以及对应设备顺序
enum errorList
{
    DBUSTOE = 0,
    YawGimbalMotorTOE,
    PitchGimbalMotorTOE,
    TriggerMotorTOE,
    ChassisMotor1TOE,
    ChassisMotor2TOE,
    ChassisMotor3TOE,
    ChassisMotor4TOE,

    errorListLength,
};

typedef __packed struct
{
    uint32_t newTime;
    uint32_t lastTime;
    uint32_t Losttime;
    uint32_t worktime;
    uint16_t setOfflineTime : 12;
    uint16_t setOnlineTime : 12;
    uint8_t enable : 1;
    uint8_t Priority : 4;
    uint8_t errorExist : 1;
    uint8_t isLost : 1;
    uint8_t dataIsError : 1;

    fp32 frequency;
    bool_t (*dataIsErrorFun)(void);
    void (*solveLostFun)(void);
    void (*solveDataErrorFun)(void);
} error_t;

extern bool_t toe_is_error(uint8_t err);
extern void Detect_task(void *pvParameters);
void DetectHook(uint8_t toe);
extern const error_t *getErrorListPoint(void);
extern void DetectHook(uint8_t toe);
#endif
