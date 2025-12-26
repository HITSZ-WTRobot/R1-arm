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

#endif //APP_H

//freertos任务定义声明

void Arm_Init(void *argument);
void Arm_control(void *argument);

//机械臂控制流程函数声明
uint8_t Arm_Rotate(float target_angel);
uint8_t Arm_Lift(float target_angel);
uint8_t Arm_Catch(float target_angel);
uint8_t Arm_Pick_Place_Process(void);


