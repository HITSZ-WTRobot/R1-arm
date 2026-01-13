/*
 * @file    app.c
 * @author  MouBowen
 * @brief   机械臂控制应用代码
 * @version 1.0
 * @date    2025-12-09
 */
#include "app.h"
#include "math.h"
#include "drivers/pump_ctrl.h"   // 气泵与电磁阀控制库
// 控制参数配置
#define ROTATE_TOLERANCE 0.05f // 旋转角度误差 (度)
#define LIFT_TOLERANCE 0.02f   // 升降角度误差 (度)
#define CATCH_TOLERANCE 0.03f  // 抓取角度误差 (度)
#define ACTION_DELAY_MS 5000    // 动作稳定延时

// 机械臂关键位置参数、
#define PICK_ROTATE_ANGLE 0.0f  // 抓取位旋转角度
#define PLACE_ROTATE_ANGLE 90.0f // 放置位旋转角度
#define PICK_LIFT_ANGLE 1.2f    // 抓取位升降角度
#define PLACE_LIFT_ANGLE 0.5f   // 放置位升降角度
#define CATCH_CLOSE_ANGLE 1.8f  // 抓取结构推出时电机旋转角度
#define CATCH_OPEN_ANGLE 0.0f   // 抓取结构收回时电机复位角
// 机械臂状态

typedef enum
{
    ARM_STATE_IDLE,            // 空闲
    ARM_STATE_ROTATE_TO_PICK,  // 旋转到抓取位
    ARM_STATE_LIFT_TO_PICK,    // 升降到抓取位
    ARM_STATE_CATCH_CLOSE,     // 抓取闭合
    ARM_STATE_LIFT_UP,         // 抓取后抬升
    ARM_STATE_ROTATE_TO_PLACE, // 旋转到放置位
    ARM_STATE_LIFT_TO_PLACE,   // 升降到放置位
    ARM_STATE_CATCH_OPEN,      // 抓取张开
    ARM_STATE_LIFT_RESET,      // 升降复位
    ARM_STATE_ROTATE_RESET,    // 旋转复位
    ARM_STATE_ERROR            // 错误
} Arm_StateTypeDef;
Arm_StateTypeDef arm_state = ARM_STATE_IDLE;

#define KEY_PRESSED_LEVEL   GPIO_PIN_RESET
#define KEY_DEBOUNCE_MS     50u

static uint8_t Key_ScanPressedEvent(GPIO_TypeDef* port, uint16_t pin)
{
    static GPIO_PinState stable13 = GPIO_PIN_SET, last13 = GPIO_PIN_SET;
    static uint32_t tchg13 = 0;
    static uint8_t latched13 = 0;

    static GPIO_PinState stable14 = GPIO_PIN_SET, last14 = GPIO_PIN_SET;
    static uint32_t tchg14 = 0;
    static uint8_t latched14 = 0;

    static GPIO_PinState stable15 = GPIO_PIN_SET, last15 = GPIO_PIN_SET;
    static uint32_t tchg15 = 0;
    static uint8_t latched15 = 0;

    GPIO_PinState* stable;
    GPIO_PinState* last;
    uint32_t* tchg;
    uint8_t* latched;

    if (pin == GPIO_PIN_13) {
        stable = &stable13; last = &last13; tchg = &tchg13; latched = &latched13;
    } else if (pin == GPIO_PIN_14) {
        stable = &stable14; last = &last14; tchg = &tchg14; latched = &latched14;
    } else if (pin == GPIO_PIN_15) {
        stable = &stable15; last = &last15; tchg = &tchg15; latched = &latched15;
    } else {
        return 0;
    }

    GPIO_PinState read = HAL_GPIO_ReadPin(port, pin);
    uint32_t now = HAL_GetTick();

    if (read != *last) { *last = read; *tchg = now; }

    if ((now - *tchg) >= KEY_DEBOUNCE_MS) {
        if (*stable != read) *stable = read;
    }

    if (*stable == KEY_PRESSED_LEVEL) {
        if (!(*latched)) { *latched = 1; return 1; }
    } else {
        *latched = 0;
    }

    return 0;
}

static inline uint8_t Key0_Pressed(void) { return Key_ScanPressedEvent(GPIOE, GPIO_PIN_13); }

typedef enum {
    arm_status_idle = 0,
    arm_status_start,
    arm_status_processing,
    arm_status_success
} arm_status_e;

static arm_status_e arm_status = arm_status_idle;
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
/*
    相关函数
 */
static uint8_t Motor_Pos_Control(Motor_PosCtrl_t *pos_ctrl, DJI_t *motor, float target_angel, float tolerance, const char *motor_name);
uint8_t Arm_Rotate(float target_angel);
uint8_t Arm_Lift(float target_angel);
uint8_t Arm_Rotate(float target_angel);
uint8_t Arm_Catch(float target_angel);
uint8_t Arm_Pick_Place_Process(void);
void motor_stop(void);

/*static void drawer_pushout_move_to_end(int dir)
{
    float vref = (dir >= 0)
                ? fabsf(drawer_pushout_params.drawer_pushout_speed_abs)
                : -fabsf(drawer_pushout_params.drawer_pushout_speed_abs);

    arm_state = arm_status_start;
;

    //__MOTOR_CTRL_DISABLE(&pos_drawer_1);
    __MOTOR_CTRL_ENABLE(&vel_catch_motor);
    hold_pos_enable = 0;
    Motor_VelCtrl_SetRef(&vel_catch_motor, vref);

    uint32_t t0 = HAL_GetTick();
    uint32_t stall_start = 0;

    while (fabsf(drawer_1.velocity) <= drawer_pushout_params.drawer_pushout_start_vel_th) {
        osDelay(1);
    }

    arm_state = arm_status_processing;

    while (1) {
        uint32_t now = HAL_GetTick();

        if ((now - t0) < drawer_pushout_params.drawer_pushout_min_run_ms) {
            stall_start = 0;
            osDelay(1);
            continue;
        }

        uint8_t stall_cond =
            (fabsf(vel_drawer_1.pid.output) >= drawer_pushout_params.drawer_pushout_stall_out_th) &&
            (fabsf(drawer_1.velocity) <= drawer_pushout_params.drawer_pushout_stall_vel_th);

        if (stall_cond) {
            if (stall_start == 0) stall_start = now;

            if ((now - stall_start) >= drawer_pushout_params.drawer_pushout_stall_confirm_ms) {
                drawer_pushout_status = drawer_pushout_status_success;
                break;
            }
        } else {
            stall_start = 0;
        }

        osDelay(1);
    }
    if(dir >= 0){
        drawer_pushout_flag = 0;
    }else{
        drawer_pushout_flag = 1;
    }
    __MOTOR_CTRL_DISABLE(&vel_drawer_1);
    Motor_VelCtrl_SetRef(&vel_drawer_1, 0.0f);
    Motor_VelCtrl_PIDReset(&vel_drawer_1);
    
    
   // __MOTOR_CTRL_ENABLE(&pos_drawer_1);
   // hold_pos_enable = 1;
   // Motor_PosCtrl_SetRef(&pos_drawer_1,drawer_1.abs_angle);
    
}*/
/* ====================== 初始化代码 ====================== */

void TIM_Callback(TIM_HandleTypeDef *htim)
{
    /**
     * 进行 PID 计算
     *
     * 只有被启用 (hctrl->enable == true) 的控制实例才会执行计算
     */
    Motor_PosCtrlUpdate(&pos_raiseandlower_motor);
    //Motor_PosCtrlCalculate(&pos_raiseandlower_motor);
    Motor_PosCtrlUpdate(&pos_catch_motor);
    Motor_VelCtrlUpdate(&vel_raiseandlower_motor);
    /**
     * 发送控制信号
     *
     * IQ_CMD_GROUP_1_4 和 IQ_CMD_GROUP_5_8 代表发送的电调 ID 组
     */
    DJI_SendSetIqCommand(&hcan2, IQ_CMD_GROUP_1_4);
    DJI_SendSetIqCommand(&hcan2, IQ_CMD_GROUP_5_8);
    //motor_stop();
}
void DJI_Control_Init()
{

    /**
     * Step0: 初始化 CAN 过滤器
     *
     * 默认使用一个过滤器 + 掩码模式
     * 亦可以使用其他过滤器模式，处理函数与使用什么过滤器无关，只要保证数据能被接收到即可
     */
    DJI_CAN_FilterInit(&hcan2, 0);
    /**
     * Step1: 注册 DJI CAN 处理回调
     *
     * 需要在 STM32CubeMX -> `Project Manager` -> `Advanced Settings`
     *  -> `Register Callback` 中启用 CAN 回调
     *
     * 一般情况下我们只使用 Fifo0，因为 Fifo0 的优先度比 Fifo1 高，当然也可以两个都使用
     */
    HAL_CAN_RegisterCallback(&hcan2, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, DJI_CAN_Fifo0ReceiveCallback);
    // HAL_CAN_RegisterCallback(&hcan2, HAL_CAN_RX_FIFO1_MSG_PENDING_CB_ID, DJI_CAN_Fifo1ReceiveCallback);

    /* Step2: 启动 CAN
     *
     * CAN 必须在注册回调后再启用，否则回调无法正常注册，同样地，我们一般也只使用 Fifo0，亦可以两个都开
     */
    CAN_Start(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
    // CAN_Start(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING);

    /**
     * Step3: 初始化电机
     *
     * auto_zero 这个参数实际上没有意义，因为 DJI 电机上电时本来就是清零了的，
     * 只是参考的学长的驱动莫名的有这个东西，所以写的时候也写了这个东西，只是发现
     * 用不上而已。
     *
     * 注意 (DJI_Config_t) 不能省去，否则会编译错误
     */
    /*DJI_Init(&rotate_motor, &(DJI_Config_t){
                                .auto_zero = false,       //< 是否在启动时自动清零角度
                                .hcan = &hcan2,           //< 电机挂载在的 CAN 句柄
                                .motor_type = M2006_C610, //< 电机类型
                                .id1 = 3,                 //< 电调 ID (1~8)
                            });*/
    DJI_Init(&raiseandlower_motor, &(DJI_Config_t){
                                       .auto_zero = false,       //< 是否在启动时自动清零角度
                                       .hcan = &hcan2,           //< 电机挂载在的 CAN 句柄
                                       .motor_type = M3508_C620, //< 电机类型
                                       .id1 = 4,                //< 电调 ID (1~8)
                                   });
    DJI_Init(&catch_motor, &(DJI_Config_t){
                               .auto_zero = false,       //< 是否在启动时自动清零角度
                               .hcan = &hcan2,           //< 电机挂载在的 CAN 句柄
                               .motor_type = M2006_C610, //< 电机类型
                               .id1 = 6,                //< 电调 ID (1~8)
                           });
    /**
     * Step4: 初始化电机控制实例
     *
     * 控制电机所需要的 PID 参数在该步传入
     * 注意：整个控制器内部并不会对输出的电流值进行限制，所以如果 abs_output_max
     *      超过限幅，输出参数很可能溢出，故 abs_output_max 必须小于或等于电机
     *      实际能接收的最大值
     */
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
                               .Kd = 0.0f,                             //
                               .abs_output_max = DJI_M2006_C610_IQ_MAX //< 限幅为电流控制最大值
                           },
                       });

    Motor_PosCtrl_Init(&pos_rotate_motor, //
                       &(Motor_PosCtrlConfig_t){
                           .motor_type = MOTOR_TYPE_DJI, //< 电机类型
                           .motor = &rotate_motor,       //< 电机nstant>, K
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

   Motor_VelCtrl_Init(&vel_raiseandlower_motor, //
                       &(Motor_VelCtrlConfig_t){
                           .motor_type = MOTOR_TYPE_DJI,  //< 电机类型
                           .motor = &raiseandlower_motor, //< 控制的电机
                           .pid = (MotorPID_Config_t){
                               .Kp = 5.0f,                           //
                               .Ki = 0.18f,                           //
                               .Kd = 0.0f,                             //
                               .abs_output_max = 8000.0f  //< 限幅为电流控制最大值
                           },
                       });
    Motor_PosCtrl_Init(&pos_raiseandlower_motor, //
                       &(Motor_PosCtrlConfig_t){
                           .motor_type = MOTOR_TYPE_DJI,  //< 电机类型
                           .motor = &raiseandlower_motor, //< 控制的电机
                           .velocity_pid = (MotorPID_Config_t){
                               .Kp = 5.0f,             //<
                               .Ki = 0.18f,             //<
                               .Kd = 0.2f,               //<
                               .abs_output_max = 8000.0f //< DJI_M3508_C620_IQ_MAX //< 限幅为电流控制最大值
                           },
                           .position_pid = (MotorPID_Config_t){
                               .Kp = 50.0f,               //<
                               .Ki = 0.08f,              //<
                               .Kd = 0.0f,              //<
                               .abs_output_max = 8000.0f //< 限速，这是外环对内环的输出限幅
                           },
                           .pos_vel_freq_ratio = 10, //< 内外环频率比（外环的频率可能需要比内环低）
                       });
    /**
     * Step5(可选): 启用或禁用控制实例
     *
     * 控制实例在初始化时默认是启动的，所以大部分情况此步可以省略。
     * 但是在有多个控制实例的情况下，必须仅保持一个控制实例开启
     */
    //__MOTOR_CTRL_ENABLE(&vel_rotate_motor);
    //__MOTOR_CTRL_DISABLE(&pos_rotate_motor);
    //__MOTOR_CTRL_ENABLE(&pos_raiseandlower_motor);
    //__MOTOR_CTRL_DISABLE(&vel_raiseandlower_motor);
    /*
     * Step6: 注册定时器回调并开启定时器
     *
     * 需要在 STM32CubeMX -> `Project Manager` -> `Advanced Settings`
     *  -> `Register Callback` 中启用 TIM 回调
     */
    HAL_TIM_RegisterCallback(&htim3, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM_Callback);
    HAL_TIM_Base_Start_IT(&htim3);
}
/**
 * 定时器回调函数，用于定时进行 PID 计算和 CAN 指令发送
 * @param htim unused
 */

void Arm_Init(void *argument)
{
    /* 初始化代码 */
     
     uint16_t target_angle = 10.0f;
     uint16_t raise_angle = 90.0f;
    DJI_Control_Init();
    arm_state = ARM_STATE_IDLE;
    __MOTOR_CTRL_DISABLE(&vel_catch_motor);
    __MOTOR_CTRL_DISABLE(&pos_catch_motor);

    
    __MOTOR_CTRL_DISABLE(&vel_raiseandlower_motor);
    __MOTOR_CTRL_DISABLE(&pos_raiseandlower_motor);


    for(;;){
        uint8_t key0 = Key0_Pressed(); 
        if(key0)
        {
            __MOTOR_CTRL_ENABLE(&pos_catch_motor);
            Motor_PosCtrl_SetRef(&pos_catch_motor, target_angle);
            target_angle += 180.0f;
            
            __MOTOR_CTRL_ENABLE(&pos_raiseandlower_motor);
            Motor_PosCtrl_SetRef(&pos_raiseandlower_motor, raise_angle);
            raise_angle += 30.0f;
        }
        osDelay(10);
    }


    //Motor_VelCtrl_SetRef(&vel_rotate_motor,60.0f);
    /* 初始化完成后退出线程 */
    osThreadExit();
}

/* ====================== 机械臂控制流程函数实现 ====================== */
static uint8_t Motor_Pos_Control(Motor_PosCtrl_t *pos_ctrl, DJI_t *motor,
                                 float target_angel, float tolerance,
                                 const char *motor_name)
{
    // 设置目标位置
    Motor_PosCtrl_SetRef(pos_ctrl, target_angel);

    while (1)
    {
        // 获取电机当前角度
        float current_pos = Motor_GetAngle(motor->motor_type, motor);

        // 误差达标判定
        if (fabs(current_pos - target_angel) < tolerance)
        {
            // printf("[%s] 到位：目标=%.2f rad，当前=%.2f rad\r\n",
            // motor_name, target_angel, current_pos);
            return 1;
        }

        osDelay(10); // 10ms轮询一次
    }
   return 1;
}

/**
 * @brief 旋转电机控制
 * @param target_angel 目标角度(degree)
 * @retval 1-成功（误差达标）
 */
uint8_t Arm_Rotate(float target_angel)
{
    return Motor_Pos_Control(&pos_rotate_motor, &rotate_motor,
                             target_angel, ROTATE_TOLERANCE, "旋转电机");
}

/**
 * @brief 升降电机控制
 * @param target_angel 目标角度(degree)
 * @retval 1-成功（误差达标）
 */
uint8_t Arm_Lift(float target_angel)
{
    return Motor_Pos_Control(&pos_raiseandlower_motor, &raiseandlower_motor,
                             target_angel, LIFT_TOLERANCE, "升降电机");
}

/**
 * @brief 抓取电机控制
 * @param target_angel 目标角(degree)
 * @retval 1-成功（误差达标）
 */
uint8_t Arm_Catch(float target_angel)
{
    return Motor_Pos_Control(&pos_catch_motor, &catch_motor,
                             target_angel, CATCH_TOLERANCE, "抓取电机");
}

/**
 * @brief 机械臂抓取-放置完整流程
 * @retval 1-流程完成
 */
uint8_t Arm_Pick_Place_Process(void)
{
    // 非空闲状态拒绝执行
    if (arm_state != ARM_STATE_IDLE)
    {
        // printf("机械臂忙，无法启动抓取放置流程！\r\n");
        return 0;
    }

    // printf("===== 启动抓取-放置流程 =====\r\n");
    arm_state = ARM_STATE_ROTATE_TO_PICK;

    // 步骤1：旋转到抓取位
    Arm_Rotate(PICK_ROTATE_ANGLE);
    arm_state = ARM_STATE_LIFT_TO_PICK;
    osDelay(ACTION_DELAY_MS);

    // 步骤2：升降到抓取高度
    Arm_Lift(PICK_LIFT_ANGLE);
    arm_state = ARM_STATE_CATCH_CLOSE;
    osDelay(ACTION_DELAY_MS);

        // 步骤3：抓取电机推出（夹紧物体）
        Arm_Catch(CATCH_CLOSE_ANGLE);

        // 电磁阀夹紧之前先启动气泵
        Pump_SetPower(75);        // 气泵功率，可根据需要调整
        osDelay(100);             // 给气泵一点建立真空的时间

        Pump_ValveOn();           // 电磁阀夹紧
        osDelay(ACTION_DELAY_MS);

    // 步骤4：抓取后抬升（使卷轴离开平台）
    arm_state = ARM_STATE_LIFT_UP;
    Arm_Lift(PLACE_LIFT_ANGLE + 0.3f);
    osDelay(ACTION_DELAY_MS);

    // 步骤5：旋转到放置位
    arm_state = ARM_STATE_ROTATE_TO_PLACE;
    Arm_Rotate(PLACE_ROTATE_ANGLE);
    osDelay(ACTION_DELAY_MS);

    // 步骤6：升降到放置高度
    arm_state = ARM_STATE_LIFT_TO_PLACE;
    Arm_Lift(PLACE_LIFT_ANGLE);
    osDelay(ACTION_DELAY_MS);

    // 步骤7：抓取电机推到指定角度（释放物体）
    arm_state = ARM_STATE_CATCH_OPEN;
    Arm_Catch(CATCH_OPEN_ANGLE);

    // 先打开电磁阀
    Pump_ValveOff();
    osDelay(100);             // 确保电磁阀已经完全打开

    // 电磁阀打开之后再停止气泵
    Pump_SetPower(0);
    osDelay(ACTION_DELAY_MS);

    // 步骤8：升降复位
    arm_state = ARM_STATE_LIFT_RESET;
    Arm_Lift(PICK_LIFT_ANGLE);
    osDelay(ACTION_DELAY_MS);

    // 步骤9：旋转复位
    arm_state = ARM_STATE_ROTATE_RESET;
    Arm_Rotate(PICK_ROTATE_ANGLE);
    osDelay(ACTION_DELAY_MS);

    // 流程完成，恢复空闲状态
    arm_state = ARM_STATE_IDLE;
    // printf("===== 抓取-放置流程完成 =====\r\n");
    return 1;
}
/**
 * @brief 机械臂控制线程（状态机驱动）
 * @param argument 线程参数
 * @retval None
 */
void Arm_Control(void *argument)
{
    //osDelay(5000); // 等待系统初始化完成
                   // printf("机械臂控制线程启动\r\n");
          
    while (1) 
    {
       
        
        /*switch (arm_state)
        {
        case ARM_STATE_IDLE:
            // 空闲状态：可通过外部触发调用 Arm_Pick_Place_Process()
            osDelay(100);
            break;

        case ARM_STATE_ERROR:
            // 错误状态：输出告警，复位
            // printf("机械臂错误状态，复位到空闲...\r\n");
            RELEASE();
            arm_state = ARM_STATE_IDLE;
            break;

        default:
            // 流程执行中，无需额外处理
            osDelay(10);
            break;
        }*/ 
    }
    //osThreadExit();
}

void motor_stop(void)
{
    if(fabsf(catch_motor.velocity) < 10.0f&&fabsf(catch_motor.iq_cmd)>1500)
    {
        Motor_VelCtrl_SetRef(&vel_catch_motor, 0.0f);
    }
}

/* ====================== 串口重定向 ====================== */
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart4, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

