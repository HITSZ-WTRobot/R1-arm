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

// 按键定义
#define ARM_CATCH_KEY_GPIO_PORT GPIOE
#define ARM_CATCH_KEY_GPIO_PIN GPIO_PIN_13
#define ARM_ROTATE_KEY_GPIO_PORT GPIOE
#define ARM_ROTATE_KEY_GPIO_PIN GPIO_PIN_14
#define ARM_RAISEANDLOWER_KEY_GPIO_PORT GPIOE
#define ARM_RAISEANDLOWER_KEY_GPIO_PIN GPIO_PIN_15

// 机械臂关键位置参数
#define ARM_RESET_ANGLE 0.0f    // 机械臂初始位置角度
#define ARM_ROTATE_SPEED 60.0f   // 抓取位旋转速度
#define ARM_CATCH_PUSH_ANGLE 50.0f   // 抓取结构推出时电机旋转角度
#define ARM_RAISEANDLOWER_HEIGHT_LOW 200.0f  // 机械臂升降低位高度
#define ARM_RAISEANDLOWER_HEIGHT_MIDDLE 800.0f // 机械臂升降中位高度
#define ARM_RAISEANDLOWER_HEIGHT_HIGH 1400.0f  // 机械臂升降高位高度

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

static uint8_t Key_GetState(GPIO_TypeDef *port, uint16_t pin)
{
    GPIO_PinState read = HAL_GPIO_ReadPin(port, pin);
    // 消抖处理
    HAL_Delay(10);
    if (HAL_GPIO_ReadPin(port, pin) != read)
        return 0;
    
    return (read == KEY_PRESSED_LEVEL) ? 1 : 0;
}

// 测试按钮 - 全部替换为按键宏
static inline uint8_t ARM_CATCH_KEY_Pressed(void)
{
    return Key_ScanPressedEvent(ARM_CATCH_KEY_GPIO_PORT, ARM_CATCH_KEY_GPIO_PIN);
}

static inline uint8_t ARM_ROTATE_KEY_IsHeld(void)
{
    return Key_GetState(ARM_ROTATE_KEY_GPIO_PORT, ARM_ROTATE_KEY_GPIO_PIN);
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
                                .id1 = 3,                 //< 电调 ID (1~8)
                            });
    DJI_Init(&raiseandlower_motor, &(DJI_Config_t){
                                       .auto_zero = false,       //< 是否在启动时自动清零角度
                                       .hcan = &hcan1,           //< 电机挂载在的 CAN 句柄
                                       .motor_type = M3508_C620, //< 电机类型
                                       .id1 = 1,                 //< 电调 ID (1~8)
                                   });
    DJI_Init(&catch_motor, &(DJI_Config_t){
                               .auto_zero = false,       //< 是否在启动时自动清零角度
                               .hcan = &hcan1,           //< 电机挂载在的 CAN 句柄
                               .motor_type = M2006_C610, //< 电机类型
                               .id1 = 4,                 //< 电调 ID (1~8)
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
                               .Kp = 5.0f,               //<
                               .Ki = 0.2f,              //<
                               .Kd = 0.50f,              //<
                               .abs_output_max = 2000.0f //< 限速，这是外环对内环的输出限幅
                           },
                           .pos_vel_freq_ratio = 10, //< 内外环频率比（外环的频率可能需要比内环低）
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
     __MOTOR_CTRL_ENABLE(&pos_catch_motor);
    __MOTOR_CTRL_DISABLE(&vel_catch_motor);
    __MOTOR_CTRL_ENABLE(&pos_raiseandlower_motor);
    __MOTOR_CTRL_DISABLE(&vel_raiseandlower_motor);
    __MOTOR_CTRL_DISABLE(&vel_rotate_motor);
    __MOTOR_CTRL_ENABLE(&pos_rotate_motor);
    
    /*
     * Step6: 注册定时器回调并开启定时器
     *
     * 需要在 STM32CubeMX -> `Project Manager` -> `Advanced Settings`
     *  -> `Register Callback` 中启用 TIM 回调
     */
    HAL_TIM_RegisterCallback(&htim6, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM_Callback);
    HAL_TIM_Base_Start_IT(&htim6);
}

/**
 * 定时器回调函数，用于定时进行 PID 计算和 CAN 指令发送
 * @param htim unused
 */
void Arm_Init(void *argument)
{
    
    /* 初始化代码 */
    static uint8_t key1_prev_state = 0; // 记录Key1上一状态
    DJI_Control_Init();
    for (;;)
    {
        uint8_t arm_catch_key = ARM_CATCH_KEY_Pressed();
        uint8_t arm_rotate_key_held = ARM_ROTATE_KEY_IsHeld();
        uint8_t arm_raiseandlower_key = ARM_RAISEANDLOWER_KEY_Pressed();
        if (arm_catch_key)
        {   
            Arm_Catch();
        }
        if (arm_rotate_key_held)
        {
            // 首次按下时切换模式
            if (key1_prev_state == 0)
            {
                Arm_Rotate();
            }
            key1_prev_state = 1;
        }
        else
        {
            // 松开时停止
            if (key1_prev_state == 1)
            {
                Arm_Rotate_Stop();
                
            }
            key1_prev_state = 0;
        }
        if (arm_raiseandlower_key)
        {
            Arm_Raiseandlower();
        }
    }
    
    // Motor_VelCtrl_SetRef(&vel_rotate_motor,60.0f);
    /* 初始化完成后退出线程 */
    osThreadExit();
}


/**
 * @brief 气泵调试运行函数运行函数
*/
void Pump_test()
{
    Pump_t pump1;
    Pump_Init(&pump1,&pump1_config);
    for(;;){
        Pump_RelayOn(&pump1);
        Pump_ValveOff(&pump1);
        osDelay(500);
        Pump_RelayOff(&pump1);
        Pump_ValveOn(&pump1);
        osDelay(500);
    }
}


/* ====================== 机械臂控制流程函数实现 ====================== */
/**
 * @brief 旋转电机控制
 * @retval 1-成功（误差达标）
 */
void Arm_Rotate()
{
    __MOTOR_CTRL_DISABLE(&pos_rotate_motor); // 关闭位置环
    __MOTOR_CTRL_ENABLE(&vel_rotate_motor);  // 开启速度环
    Motor_VelCtrl_SetRef(&vel_rotate_motor, ARM_ROTATE_SPEED); // 设置转速
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
void Arm_Raiseandlower()
{
    static int height_level = 0;
    height_level++;
    
    switch (height_level)
    {
        case 1:
            Motor_PosCtrl_SetRef(&pos_raiseandlower_motor, ARM_RAISEANDLOWER_HEIGHT_LOW);
            break;
        case 2:
            Motor_PosCtrl_SetRef(&pos_raiseandlower_motor, ARM_RAISEANDLOWER_HEIGHT_MIDDLE);
            break;
        case 3:
            Motor_PosCtrl_SetRef(&pos_raiseandlower_motor, ARM_RAISEANDLOWER_HEIGHT_HIGH);
            break;
        default:
            height_level = 0;
            Motor_PosCtrl_SetRef(&pos_raiseandlower_motor, ARM_RESET_ANGLE);
            break;
    }
}

/**
 * @brief 抓取电机控制
 * @param target_angel 目标角(degree)
 * @retval 1-成功（误差达标）
 */
void Arm_Catch()
{
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
