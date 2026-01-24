/*
 * @file    app.c
 * @author  MouBowen
 * @brief   机械臂控制应用代码
 * @version 1.0
 * @date    2025-12-09
 */
#include "app.h"
#include "math.h"
#include "pump_ctrl.h"

// 气泵与电磁阀配置定义
#define PUMP_VALVE_GPIO_Port GPIOE
#define PUMP_VALVE_Pin GPIO_PIN_3
#define PUMP_RELAY_GPIO_Port GPIOE
#define PUMP_RELAY_Pin GPIO_PIN_4



Pump_Config_t pump1_config = {
    .htim = &htim3,
    .channel = TIM_CHANNEL_1,
    .valve_port = PUMP_VALVE_GPIO_Port,
    .valve_pin = PUMP_VALVE_Pin,
    .pump_port = PUMP_RELAY_GPIO_Port,
    .pump_pin = PUMP_RELAY_Pin,
    .invert = 1
};

// 全局气泵句柄实例，供各控制流程复用
Pump_t pump1;

// 按键定义
#define ARM_CATCH_KEY_GPIO_PORT GPIOE
#define ARM_CATCH_KEY_GPIO_PIN GPIO_PIN_13
#define ARM_ROTATE_KEY_GPIO_PORT GPIOE
#define ARM_ROTATE_KEY_GPIO_PIN GPIO_PIN_14
#define ARM_RAISEANDLOWER_KEY_GPIO_PORT GPIOE
#define ARM_RAISEANDLOWER_KEY_GPIO_PIN GPIO_PIN_15

// 机械臂关键位置参数
#define ARM_RESET_ANGLE 0.0f    // 机械臂初始位置角度

//CATCH: 抓取与释放角度参数
#define ARM_CATCH_PUSH_ANGLE 250.0f   // 抓取结构推出时电机旋转角度
#define ARM_CATCH_PUSH_ANGLE_MAX 280.0f  // 抓取结构推出最大角度

//HEIGHT: 抓取与释放高度参数
#define ARM_CATCH_HEIGHT_LOW 0.0f     //抓取200高度卷轴需要的抬升高度
#define ARM_RELEASE_HEIGHT_LOW 1100.0f   //释放200高度卷轴需要的抬升高度
#define ARM_CATCH_HEIGHT_MID 700.0f    //抓取400高度卷轴需要的抬升高度
#define ARM_RELEASE_HEIGHT_MID 1100.0f  //释放400高度卷轴需要的抬升高度
#define ARM_CATCH_HEIGHT_HIGH 1350.0f  //抓取600高度卷轴需要的抬升高度
#define ARM_RELEASE_HEIGHT_HIGH 1100.0f //释放600高度卷轴需要的抬升高度

//ROTATE_ANGLE: 抓取与释放旋转角度参数
#define ARM_ROTATE_ANGLE -320.0f     //旋转转轴到存放需要的旋转角度

// 按键扫描相关代码
#define KEY_PRESSED_LEVEL GPIO_PIN_RESET
#define KEY_DEBOUNCE_MS 50u

static uint8_t Key_ScanPressedEvent(GPIO_TypeDef *port, uint16_t pin)
{
    // 替换为宏对应的静态变量命名，增强可读性
    static GPIO_PinState stable_catch = GPIO_PIN_SET, last_catch = GPIO_PIN_SET;
    static uint32_t tchg_catch = 0;
    static uint8_t latched_catch = 0;

    static GPIO_PinState stable_rotate = GPIO_PIN_SET, last_rotate = GPIO_PIN_SET;
    static uint32_t tchg_rotate = 0;
    static uint8_t latched_rotate = 0;

    static GPIO_PinState stable_raiseandlower = GPIO_PIN_SET, last_raiseandlower = GPIO_PIN_SET;
    static uint32_t tchg_raiseandlower = 0;
    static uint8_t latched_raiseandlower = 0;

    GPIO_PinState *stable;
    GPIO_PinState *last;
    uint32_t *tchg;
    uint8_t *latched;

    // 使用按键宏进行判断，替换硬编码的引脚值
    if (pin == ARM_CATCH_KEY_GPIO_PIN)
    {
        stable = &stable_catch;
        last = &last_catch;
        tchg = &tchg_catch;
        latched = &latched_catch;
    }
    else if (pin == ARM_ROTATE_KEY_GPIO_PIN)
    {
        stable = &stable_rotate;
        last = &last_rotate;
        tchg = &tchg_rotate;
        latched = &latched_rotate;
    }
    else if (pin == ARM_RAISEANDLOWER_KEY_GPIO_PIN)
    {
        stable = &stable_raiseandlower;
        last = &last_raiseandlower;
        tchg = &tchg_raiseandlower;
        latched = &latched_raiseandlower;
    }
    else
    {
        return 0;
    }

    GPIO_PinState read = HAL_GPIO_ReadPin(port, pin);
    uint32_t now = HAL_GetTick();

    if (read != *last)
    {
        *last = read;
        *tchg = now;
    }

    if ((now - *tchg) >= KEY_DEBOUNCE_MS)
    {
        if (*stable != read)
            *stable = read;
    }

    if (*stable == KEY_PRESSED_LEVEL)
    {
        if (!(*latched))
        {
            *latched = 1;
            return 1;
        }
    }
    else
    {
        *latched = 0;
    }

    return 0;
}

// 测试按钮 - 全部替换为按键宏
static inline uint8_t ARM_CATCH_KEY_Pressed(void)
{
    return Key_ScanPressedEvent(ARM_CATCH_KEY_GPIO_PORT, ARM_CATCH_KEY_GPIO_PIN);
}

static inline uint8_t ARM_ROTATE_KEY_Pressed(void)
{
    return Key_ScanPressedEvent(ARM_ROTATE_KEY_GPIO_PORT, ARM_ROTATE_KEY_GPIO_PIN);
}

static inline uint8_t ARM_RAISEANDLOWER_KEY_Pressed(void)
{
    return Key_ScanPressedEvent(ARM_RAISEANDLOWER_KEY_GPIO_PORT, ARM_RAISEANDLOWER_KEY_GPIO_PIN);
}

DJI_t rotate_motor;
DJI_t raiseandlower_motor;
DJI_t catch_motor;

/**
 * 位置环控制实例
 */
Motor_PosCtrl_t pos_rotate_motor;
Motor_PosCtrl_t pos_raiseandlower_motor;
Motor_PosCtrl_t pos_catch_motor;

/**
 * 速度环控制实例
 */
Motor_VelCtrl_t vel_rotate_motor;
Motor_VelCtrl_t vel_raiseandlower_motor;
Motor_VelCtrl_t vel_catch_motor;

// 取卷轴流程步进标志与当前步骤
static volatile uint8_t g_pick_step_trigger = 0; // 由按键置 1，流程函数读到后清零并前进一步
static uint8_t         g_pick_step         = 0; // 当前执行到的步骤编号

// 简单阻塞等待位置环就位，带超时保护
static void Arm_WaitPosSettle(Motor_PosCtrl_t *hctrl, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    while (!Motor_PosCtrl_IsSettle(hctrl))
    {
        if ((HAL_GetTick() - start) > timeout_ms)
        {
            break; // 超时直接退出，防止死等
        }
        osDelay(5);
    }
}


/* ====================== 初始化代码 ====================== */
void TIM_Callback(TIM_HandleTypeDef *htim)
{
    /**
     * 进行 PID 计算
     *
     * 只有被启用 (hctrl->enable == true) 的控制实例才会执行计算
     */
    Motor_PosCtrlUpdate(&pos_raiseandlower_motor);
    Motor_PosCtrlUpdate(&pos_catch_motor);
    Motor_PosCtrlUpdate(&pos_rotate_motor);
    Motor_VelCtrlUpdate(&vel_raiseandlower_motor);
    Motor_VelCtrlUpdate(&vel_rotate_motor);
    Motor_VelCtrlUpdate(&vel_catch_motor);
    
    /**
     * 发送控制信号
     *
     * IQ_CMD_GROUP_1_4 和 IQ_CMD_GROUP_5_8 代表发送的电调 ID 组
     */
    //DJI_SendSetIqCommand(&hcan1, IQ_CMD_GROUP_1_4);
    // DJI_SendSetIqCommand(&hcan1, IQ_CMD_GROUP_5_8);
    DJI_SendSetIqCommand(&hcan2, IQ_CMD_GROUP_1_4);
    DJI_SendSetIqCommand(&hcan2, IQ_CMD_GROUP_5_8);
}

void DJI_Control_Init()
{
    DJI_CAN_FilterInit(&hcan1, 0);
    DJI_CAN_FilterInit(&hcan2, 14);
    
    HAL_CAN_RegisterCallback(&hcan1, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, DJI_CAN_Fifo0ReceiveCallback);
    HAL_CAN_RegisterCallback(&hcan2, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, DJI_CAN_Fifo0ReceiveCallback);
    
    CAN_Start(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    CAN_Start(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
    
    DJI_Init(&rotate_motor, &(DJI_Config_t){
                                .auto_zero = false,       //< 是否在启动时自动清零角度
                                .hcan = &hcan2,           //< 电机挂载在的 CAN 句柄
                                .motor_type = M2006_C610, //< 电机类型
                                .id1 = 1,                 //< 电调 ID (1~8)
                            });
    DJI_Init(&raiseandlower_motor, &(DJI_Config_t){
                                       .auto_zero = false,       //< 是否在启动时自动清零角度
                                       .hcan = &hcan2,           //< 电机挂载在的 CAN 句柄
                                       .motor_type = M3508_C620, //< 电机类型
                                       .id1 = 3,                 //< 电调 ID (1~8)
                                   });
    DJI_Init(&catch_motor, &(DJI_Config_t){
                               .auto_zero = false,       //< 是否在启动时自动清零角度
                               .hcan = &hcan2,           //< 电机挂载在的 CAN 句柄
                               .motor_type = M2006_C610, //< 电机类型
                               .id1 = 2,                 //< 电调 ID (1~8)
                           });
   
    Motor_VelCtrl_Init(&vel_catch_motor, //
                       &(Motor_VelCtrlConfig_t){
                           .motor_type = MOTOR_TYPE_DJI, //< 电机类型
                           .motor = &catch_motor,        //< 控制的电机
                           .pid = (MotorPID_Config_t){
                               .Kp = 200.0f,                           //
                               .Ki = 0.05f,                            //
                               .Kd = 0.0f,                             //
                               .abs_output_max = DJI_M2006_C610_IQ_MAX //< 限幅为电流控制最大值
                           },
                       });

    Motor_PosCtrl_Init(&pos_catch_motor, //
                       &(Motor_PosCtrlConfig_t){
                           .motor_type = MOTOR_TYPE_DJI, //< 电机类型
                           .motor = &catch_motor,        //< 电机nstant>, K
                           .velocity_pid = (MotorPID_Config_t){
                               .Kp = 100.0f,
                               .Ki = 0.5f,                             //<
                               .Kd = 0.9f,                             //<
                               .abs_output_max = DJI_M2006_C610_IQ_MAX //< DJI_M3508_C620_IQ_MAX //< 限幅为电流控制最大值
                           },
                           .position_pid = (MotorPID_Config_t){
                               .Kp = 1.0f,              //<
                               .Ki = 0.002f,            //<
                               .Kd = 0.8f,              //<
                               .abs_output_max = 500.0f //< 限速，这是外环对内环的输出限幅
                           },
                           .pos_vel_freq_ratio = 1, //< 内外环频率比（外环的频率可能需要比内环低）
                       });

    Motor_VelCtrl_Init(&vel_rotate_motor, //
                       &(Motor_VelCtrlConfig_t){
                           .motor_type = MOTOR_TYPE_DJI, //< 电机类型
                           .motor = &rotate_motor,       //< 控制的电机
                           .pid = (MotorPID_Config_t){
                               .Kp = 200.0f,                           //
                               .Ki = 0.05f,                            //
                               .Kd = 0.0f,                        //
                               .abs_output_max = 8000.0f //< 限幅为电流控制最大值
                           },
                       });

    Motor_PosCtrl_Init(&pos_rotate_motor, //
                       &(Motor_PosCtrlConfig_t){
                           .motor_type = MOTOR_TYPE_DJI, //< 电机类型
                           .motor = &rotate_motor,       //< 电机nstant>, K
                           .velocity_pid = (MotorPID_Config_t){
                               .Kp = 100.0f,
                               .Ki = 0.5f,                             //<
                               .Kd = 0.9f,                                //<
                               .abs_output_max = 8000.0f //< DJI_M3508_C620_IQ_MAX //< 限幅为电流控制最大值
                           },
                           .position_pid = (MotorPID_Config_t){
                               .Kp = 24.5f,              //<
                               .Ki = 0.42f,            //<
                               .Kd = 100.0f,              //<
                               .abs_output_max = 500.0f //< 限速，这是外环对内环的输出限幅
                           },
                           .pos_vel_freq_ratio = 10, //< 内外环频率比（外环的频率可能需要比内环低）
                       });

    Motor_VelCtrl_Init(&vel_raiseandlower_motor, //
                       &(Motor_VelCtrlConfig_t){
                           .motor_type = MOTOR_TYPE_DJI,  //< 电机类型
                           .motor = &raiseandlower_motor, //< 控制的电机
                           .pid = (MotorPID_Config_t){
                               .Kp = 5.0f,               //
                               .Ki = 0.18f,              //
                               .Kd = 0.0f,               //
                               .abs_output_max = 8000.0f //< 限幅为电流控制最大值
                           },
                       });

    Motor_PosCtrl_Init(&pos_raiseandlower_motor, //
                       &(Motor_PosCtrlConfig_t){
                           .motor_type = MOTOR_TYPE_DJI,  //< 电机类型
                           .motor = &raiseandlower_motor, //< 控制的电机
                           .velocity_pid = (MotorPID_Config_t){
                               .Kp = 100.0f,             //<
                               .Ki = 0.001f,             //<
                               .Kd = 0.5f,               //<
                               .abs_output_max = 8000.0f //< DJI_M3508_C620_IQ_MAX //< 限幅为电流控制最大值
                           },
                           .position_pid = (MotorPID_Config_t){
                               .Kp = 2.0f,               //<
                               .Ki = 0.01f,              //<
                               .Kd = 0.20f,              //<
                               .abs_output_max = 2000.0f //< 限速，这是外环对内环的输出限幅
                           },
                           .pos_vel_freq_ratio = 1, //< 内外环频率比（外环的频率可能需要比内环低）
                       });
    
    /**
     * Step5(可选): 启用或禁用控制实例
     *
     * 控制实例在初始化时默认是启动的，所以大部分情况此步可以省略。
     * 但是在有多个控制实例的情况下，必须仅保持一个控制实例开启
     */
    // 初始化阶段不启用任何位置/速度环控制，
    // 避免上电后电机立即锁死在某个初始位置。
    __MOTOR_CTRL_DISABLE(&pos_catch_motor);
    __MOTOR_CTRL_DISABLE(&vel_catch_motor);
    __MOTOR_CTRL_DISABLE(&pos_raiseandlower_motor);
    __MOTOR_CTRL_DISABLE(&vel_raiseandlower_motor);
    __MOTOR_CTRL_DISABLE(&vel_rotate_motor);
    __MOTOR_CTRL_DISABLE(&pos_rotate_motor);
    
    /*
     * Step6: 注册定时器回调并开启定时器
     *
     * 需要在 STM32CubeMX -> `Project Manager` -> `Advanced Settings`
     *  -> `Register Callback` 中启用 TIM 回调
     */
    HAL_TIM_RegisterCallback(&htim6, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM_Callback);
    HAL_TIM_Base_Start_IT(&htim6);
}

float arm_height = 0.0f;
float arm_rotate_angle = 0.0f;
float arm_catch_angle = 0.0f;
/**
 * 定时器回调函数，用于定时进行 PID 计算和 CAN 指令发送
 * @param htim unused
 */

//调试代码
// void Arm_Init(void *argument)
// {
//     /* 初始化代码 */
//     DJI_Control_Init();
//     Pump_Init(&pump1,&pump1_config);
//     for (;;)
//     {
//         uint8_t arm_catch_key = ARM_CATCH_KEY_Pressed();
//         uint8_t arm_rotate_key = ARM_ROTATE_KEY_Pressed();
//         uint8_t arm_raiseandlower_key = ARM_RAISEANDLOWER_KEY_Pressed();
//         if (arm_catch_key)
//         {   
//             // 1 表示本次允许执行抓取/旋转等动作
//             Pump_Catch(&pump1, 1);
//             //Arm_Catch(1);
//         }
//         if (arm_rotate_key)
//         {
//             Pump_Release(&pump1, 1);
//             //Arm_Rotate(1);
//         }
//         if (arm_raiseandlower_key)
//         {
//             Motor_PosCtrl_SetRef(&pos_raiseandlower_motor, arm_height);
//             Motor_PosCtrl_SetRef(&pos_rotate_motor, arm_rotate_angle);
//             Motor_PosCtrl_SetRef(&pos_catch_motor, arm_catch_angle);
//         }
//     }
//     // Motor_VelCtrl_SetRef(&vel_rotate_motor,60.0f);
//     /* 初始化完成后退出线程 */
//     osThreadExit();
// }

void Arm_Init(void *argument)
{
    
    /* 初始化代码 */
    DJI_Control_Init();
    Pump_Init(&pump1,&pump1_config);
    for (;;)
    {
        uint8_t step_key  = ARM_CATCH_KEY_Pressed();       // 步进按键
        uint8_t reset_key = ARM_ROTATE_KEY_Pressed();      // 复位按键
        uint8_t null_key  = ARM_RAISEANDLOWER_KEY_Pressed(); // 预留按键（当前未用）

        // 步进：每按一次步进键，让流程前进一步
        if (step_key)
        {
            g_pick_step_trigger = 1;
        }

        // 复位：清空流程状态，并将机械臂复位
        if (reset_key)
        {
            g_pick_step        = 0;
            g_pick_step_trigger = 0;

            // 复位电机与气泵
            Motor_PosCtrl_SetRef(&pos_raiseandlower_motor, ARM_RESET_ANGLE);
            Motor_PosCtrl_SetRef(&pos_catch_motor, ARM_RESET_ANGLE);
            Motor_PosCtrl_SetRef(&pos_rotate_motor, ARM_RESET_ANGLE);
            Pump_Release(&pump1, 1);
        }

        // 预留按键目前不做处理，保留扩展空间
        (void)null_key;

        // 默认使用“高卷轴”流程做测试，如果需要中/低流程，可在此改为
        // Arm_PickMid() 或 Arm_PickLow()，或在此处根据模式切换调用。
        Arm_PickHigh();

        osDelay(5);
    }
    
    // Motor_VelCtrl_SetRef(&vel_rotate_motor,60.0f);
    /* 初始化完成后退出线程 */
    osThreadExit();
}


/* ====================== 机械臂控制流程函数实现 ====================== */
/**
 * @brief 旋转电机控制
 * @retval 1-成功（误差达标）
 */
void Arm_Rotate(uint8_t enable)
{    
    if (!enable)
    {
        return;
    }
    // 在执行旋转动作前启用对应位置环控制
    __MOTOR_CTRL_ENABLE(&pos_rotate_motor);

    static uint8_t rotate_state = 0;
    rotate_state++;
    switch (rotate_state)
    {
         case 1:
              Motor_PosCtrl_SetRef(&pos_rotate_motor, ARM_ROTATE_ANGLE);
              break;
         default:
              rotate_state = 0;
              Motor_PosCtrl_SetRef(&pos_rotate_motor, ARM_RESET_ANGLE);
              break;
    }
}


void Arm_Rotate_Stop()
{
    Motor_VelCtrl_SetRef(&vel_rotate_motor, 0.0f); // 速度归零
                
    // 切回位置环并锁定当前位置
    __MOTOR_CTRL_DISABLE(&vel_rotate_motor);
    __MOTOR_CTRL_ENABLE(&pos_rotate_motor);
    // 更新位置环目标值为当前电机角度，防止电机突然回退
    Motor_ResetAngle(MOTOR_TYPE_DJI, &rotate_motor);
    Motor_PosCtrl_SetRef(&pos_rotate_motor, 0.0f);
}

/**
 * @brief 升降电机控制
 * @param target_angel 目标角度(degree)
 * @retval 1-成功（误差达标）
 */
void Arm_Raiseandlower(uint8_t enable)
{
    if (!enable)
    {
        return;
    }
    // 在执行升降动作前启用对应位置环控制
    __MOTOR_CTRL_ENABLE(&pos_raiseandlower_motor);
    static int height_level = 0;
    height_level++;
    
    switch (height_level)
    {
        case 1:
            Motor_PosCtrl_SetRef(&pos_raiseandlower_motor, ARM_CATCH_HEIGHT_LOW);
            break;
        case 2:
            Motor_PosCtrl_SetRef(&pos_raiseandlower_motor, ARM_CATCH_HEIGHT_MID);
            break;
        case 3:
            Motor_PosCtrl_SetRef(&pos_raiseandlower_motor, ARM_CATCH_HEIGHT_HIGH);
            break;
        default:
            height_level = 0;
            Motor_PosCtrl_SetRef(&pos_raiseandlower_motor, ARM_RESET_ANGLE);
            break;
    }
}

// /**
//  * @brief 升降电机每次在当前位置基础上增加 50 度
//  * @note  不改变原有高度分档逻辑，仅提供额外的步进控制接口
//  */
// void Arm_Raiseandlower_Step50()
// {
//     float current_angle = MotorCtrl_GetAngle(&pos_raiseandlower_motor);
//     Motor_PosCtrl_SetRef(&pos_raiseandlower_motor, current_angle + 50.0f);
// }

/**
 * @brief 抓取电机控制
 * @param target_angel 目标角(degree)
 * @retval 1-成功（误差达标）
 */
void Arm_Catch(uint8_t enable)
{
    if (!enable)
    {
        return;
    }
    // 在执行抓取动作前启用对应位置环控制
    __MOTOR_CTRL_ENABLE(&pos_catch_motor);
    static uint8_t rotate_state = 0;
   rotate_state++;
    switch (rotate_state)
    {
         case 1:
              Motor_PosCtrl_SetRef(&pos_catch_motor, ARM_CATCH_PUSH_ANGLE);
              break;
         default:
              rotate_state = 0;
              Motor_PosCtrl_SetRef(&pos_catch_motor, ARM_RESET_ANGLE);
              break;
    }
}

/* ====================== 取卷轴一键流程（步进状态机） ====================== */

// 核心状态机：根据当前高度档和步进标志，执行下一步
static void Arm_PickCore(Arm_PickLevel_t level)
{
    if (!g_pick_step_trigger)
    {
        return; // 没有收到步进标志，不做任何动作
    }

    // 消费本次步进标志
    g_pick_step_trigger = 0;

    // 确保相关位置环启用
    __MOTOR_CTRL_ENABLE(&pos_raiseandlower_motor);
    __MOTOR_CTRL_ENABLE(&pos_catch_motor);

    float catch_height   = 0.0f;
    float release_height = 0.0f;

    switch (level)
    {
    case ARM_PICK_LEVEL_HIGH:
        catch_height   = ARM_CATCH_HEIGHT_HIGH;
        release_height = ARM_RELEASE_HEIGHT_HIGH;
        break;
    case ARM_PICK_LEVEL_MID:
        catch_height   = ARM_CATCH_HEIGHT_MID;
        release_height = ARM_RELEASE_HEIGHT_MID;
        break;
    case ARM_PICK_LEVEL_LOW:
    default:
        catch_height   = ARM_CATCH_HEIGHT_LOW;
        release_height = ARM_RELEASE_HEIGHT_LOW;
        break;
    }

    switch (g_pick_step)
    {
    case 0:
        // 1. 升降到对应“抓取高度”
        Motor_PosCtrl_SetRef(&pos_raiseandlower_motor, catch_height);
        g_pick_step = 1;
        break;

    case 1:
        // 2. 开启气泵抓取
        Pump_Catch(&pump1, 1);
        g_pick_step = 2;
        break;

    case 2:
        // 3. 推出抓取机械臂
        Motor_PosCtrl_SetRef(&pos_catch_motor, ARM_CATCH_PUSH_ANGLE);
        g_pick_step = 3;
        break;

    case 3:
        // 4. 收回抓取机械臂
        Motor_PosCtrl_SetRef(&pos_catch_motor, ARM_RESET_ANGLE);
        g_pick_step = 4;
        break;

    case 4:
        // 5. 降到对应“释放高度”
        Motor_PosCtrl_SetRef(&pos_raiseandlower_motor, release_height);
        g_pick_step = 5;
        break;

    case 5:
        // 6. 关闭气泵 / 释放
        Pump_Release(&pump1, 1);
        g_pick_step = 6;
        break;

    case 6:
        // 7. 全部复位
        Motor_PosCtrl_SetRef(&pos_raiseandlower_motor, ARM_RESET_ANGLE);
        Motor_PosCtrl_SetRef(&pos_catch_motor, ARM_RESET_ANGLE);
        Motor_PosCtrl_SetRef(&pos_rotate_motor, ARM_RESET_ANGLE);
        g_pick_step = 0; // 流程结束，回到初始步骤
        break;

    default:
        g_pick_step = 0;
        break;
    }
}

// 取较高卷轴（600 高度）
void Arm_PickHigh(void)
{
    Arm_PickCore(ARM_PICK_LEVEL_HIGH);
}

// 取中等高度卷轴（400 高度）
void Arm_PickMid(void)
{
    Arm_PickCore(ARM_PICK_LEVEL_MID);
}

// 取较低卷轴（200 高度）
void Arm_PickLow(void)
{
    Arm_PickCore(ARM_PICK_LEVEL_LOW);
}
