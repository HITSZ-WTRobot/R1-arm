/**
 * @file    app.h
 * @author  syhanjin
 * @date    2025-09-20
 */
#ifndef APP_H
#define APP_H

/* Includes */
#include "libs/printf.h"
#include "stdio.h"
#include "bsp/can_driver.h"
#include "can.h"
#include "drivers/DJI.h"
#include "interfaces/motor_if.h"
#include "tim.h"
#include "bsp/gpio_driver.h"
#include "cmsis_os2.h"
#include "usart.h"

/*
    机械臂控制相关函数
 */
void Arm_Rotate(); // 机械臂旋转控制函数
void Arm_Rotate_Stop();// 机械臂旋转停止函数
void  Arm_Raiseandlower();// 机械臂升降控制函数
void  Arm_Catch();// 机械臂抓取控制函数
void  Arm_Raiseandlower_Step50();// 升降电机在当前位置基础上每次加 50 度



#endif //APP_H
