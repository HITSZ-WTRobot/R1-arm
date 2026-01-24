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

// 取卷轴高度档位
typedef enum
{
    ARM_PICK_LEVEL_LOW = 0,  // 低卷轴（200 高度）
    ARM_PICK_LEVEL_MID,      // 中卷轴（400 高度）
    ARM_PICK_LEVEL_HIGH,     // 高卷轴（600 高度）
} Arm_PickLevel_t;

/*
    机械臂控制相关函数
    enable 标志位：1 执行，0 不执行
 */
void Arm_Rotate(uint8_t enable); // 机械臂旋转控制函数
void Arm_Rotate_Stop();// 机械臂旋转停止函数
void  Arm_Raiseandlower(uint8_t enable);// 机械臂升降控制函数
void  Arm_Catch(uint8_t enable);// 机械臂抓取控制函数



// 一键取卷轴流程：高 / 中 / 低
void Arm_PickHigh(void);
void Arm_PickMid(void);
void Arm_PickLow(void);

// 遥控器/上层控制调用接口（输入均为 1/0 或枚举）
Arm_PickLevel_t Arm_SetLevelRC(Arm_PickLevel_t level); // 设置 LEVEL 档位，若流程未完成则保持原档位并返回当前值
void Arm_StepRC(uint8_t step);              // STEP 输入：1 表示本周期按下，0 表示松开
void Arm_ResetRC(uint8_t reset);            // RESET 输入：1 表示本周期按下，0 表示松开
uint8_t Arm_GetStateRC(void);               // 获取当前步骤编号，便于显示/调试


#endif //APP_H
