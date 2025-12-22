/*
 * @file    app.c
 * @author  MouBowen
 * @brief   机械臂控制应用代码
 * @version 1.0
 * @date    2025-12-09
 */
#include "app.h"
#include "math.h"
// 电磁阀控制引脚定义
#define SOLENOID_VALVE_GPIO_PORT GPIOA
#define SOLENOID_VALVE_GPIO_PIN GPIO_PIN_0

// 电磁阀控制宏
#define GRIP() HAL_GPIO_WritePin(SOLENOID_VALVE_GPIO_PORT, SOLENOID_VALVE_GPIO_PIN, GPIO_PIN_SET)      // 打开（高电平）
#define RELEASE() HAL_GPIO_WritePin(SOLENOID_VALVE_GPIO_PORT, SOLENOID_VALVE_GPIO_PIN, GPIO_PIN_RESET) // 关闭（低电平）
#define TOGGLE() HAL_GPIO_TogglePin(SOLENOID_VALVE_GPIO_PORT, SOLENOID_VALVE_GPIO_PIN)                 // 翻转状态

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
/* ====================== 初始化代码 ====================== */

void TIM_Callback(TIM_HandleTypeDef *htim)
{
    /**
     * 进行 PID 计算
     *
     * 只有被启用 (hctrl->enable == true) 的控制实例才会执行计算
     */
    Motor_PosCtrlCalculate(&pos_rotate_motor);
    Motor_PosCtrlCalculate(&pos_raiseandlower_motor);
    Motor_PosCtrlCalculate(&pos_catch_motor);
    /**
     * 发送控制信号
     *
     * IQ_CMD_GROUP_1_4 和 IQ_CMD_GROUP_5_8 代表发送的电调 ID 组
     */
    DJI_SendSetIqCommand(&hcan1, IQ_CMD_GROUP_1_4);
    // DJI_SendSetIqCommand(&hcan1, IQ_CMD_GROUP_5_8);
}
void DJI_Control_Init()
{

    /**
     * Step0: 初始化 CAN 过滤器
     *
     * 默认使用一个过滤器 + 掩码模式
     * 亦可以使用其他过滤器模式，处理函数与使用什么过滤器无关，只要保证数据能被接收到即可
     */
    DJI_CAN_FilterInit(&hcan1, 0);

    /**
     * Step1: 注册 DJI CAN 处理回调
     *
     * 需要在 STM32CubeMX -> `Project Manager` -> `Advanced Settings`
     *  -> `Register Callback` 中启用 CAN 回调
     *
     * 一般情况下我们只使用 Fifo0，因为 Fifo0 的优先度比 Fifo1 高，当然也可以两个都使用
     */
    HAL_CAN_RegisterCallback(&hcan1, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, DJI_CAN_Fifo0ReceiveCallback);
    // HAL_CAN_RegisterCallback(&hcan1, HAL_CAN_RX_FIFO1_MSG_PENDING_CB_ID, DJI_CAN_Fifo1ReceiveCallback);

    /* Step2: 启动 CAN
     *
     * CAN 必须在注册回调后再启用，否则回调无法正常注册，同样地，我们一般也只使用 Fifo0，亦可以两个都开
     */
    CAN_Start(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    // CAN_Start(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING);

    /**
     * Step3: 初始化电机
     *
     * auto_zero 这个参数实际上没有意义，因为 DJI 电机上电时本来就是清零了的，
     * 只是参考的学长的驱动莫名的有这个东西，所以写的时候也写了这个东西，只是发现
     * 用不上而已。
     *
     * 注意 (DJI_Config_t) 不能省去，否则会编译错误
     */
    DJI_Init(&rotate_motor, (DJI_Config_t){
                                .auto_zero = false,       //< 是否在启动时自动清零角度
                                .hcan = &hcan1,           //< 电机挂载在的 CAN 句柄
                                .motor_type = M2006_C610, //< 电机类型
                                .id1 = 1,                 //< 电调 ID (1~8)
                            });
    DJI_Init(&raiseandlower_motor, (DJI_Config_t){
                                       .auto_zero = false,       //< 是否在启动时自动清零角度
                                       .hcan = &hcan1,           //< 电机挂载在的 CAN 句柄
                                       .motor_type = M3508_C620, //< 电机类型
                                       .id1 = 2,                 //< 电调 ID (1~8)
                                   });
    DJI_Init(&catch_motor, (DJI_Config_t){
                               .auto_zero = false,       //< 是否在启动时自动清零角度
                               .hcan = &hcan1,           //< 电机挂载在的 CAN 句柄
                               .motor_type = M2006_C610, //< 电机类型
                               .id1 = 3,                 //< 电调 ID (1~8)
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
                       (Motor_VelCtrlConfig_t){
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
                       (Motor_PosCtrlConfig_t){
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
                       (Motor_VelCtrlConfig_t){
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
                       (Motor_PosCtrlConfig_t){
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
                       (Motor_VelCtrlConfig_t){
                           .motor_type = MOTOR_TYPE_DJI,  //< 电机类型
                           .motor = &raiseandlower_motor, //< 控制的电机
                           .pid = (MotorPID_Config_t){
                               .Kp = 100.0f,                           //
                               .Ki = 0.001f,                           //
                               .Kd = 0.5f,                             //
                               .abs_output_max = DJI_M3508_C620_IQ_MAX //< 限幅为电流控制最大值
                           },
                       });
    /**
     * 如之前所言：同一个电机可以有不同的控制实例，故这两个初始化都可以传入同一个电机
     */
    Motor_PosCtrl_Init(&pos_raiseandlower_motor, //
                       (Motor_PosCtrlConfig_t){
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
    __MOTOR_CTRL_DISABLE(&vel_rotate_motor);
    __MOTOR_CTRL_ENABLE(&pos_catch_motor);
    __MOTOR_CTRL_DISABLE(&vel_raiseandlower_motor);
    __MOTOR_CTRL_ENABLE(&pos_raiseandlower_motor);
    __MOTOR_CTRL_DISABLE(&vel_catch_motor);
    __MOTOR_CTRL_ENABLE(&pos_catch_motor);
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
    DJI_Control_Init();
    arm_state = ARM_STATE_IDLE;
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
    GRIP(); // 电磁阀夹紧
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
    RELEASE(); // 电磁阀释放
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
void Arm_control(void *argument)
{
    osDelay(5000); // 等待系统初始化完成
                   // printf("机械臂控制线程启动\r\n");
    arm_state = ARM_STATE_ROTATE_TO_PLACE;
    Arm_Rotate(-2260.0f);
    osDelay(ACTION_DELAY_MS);
    arm_state = ARM_STATE_ROTATE_RESET;
    Arm_Rotate(PICK_ROTATE_ANGLE);
    osDelay(ACTION_DELAY_MS);
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
       break;
    }
}

/* ====================== 串口重定向 ====================== */
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart6, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/* ====================== 加入视觉识别需增加（改变） ====================== */
   /* #define BTN1_GPIO_PORT  GPIOB
    #define BTN1_GPIO_PIN   GPIO_PIN_0  // 按钮1：启动抓取流程
    define BTN2_GPIO_PORT  GPIOB
    #define BTN2_GPIO_PIN   GPIO_PIN_1  // 按钮2：启动放箱流程
    typedef enum {
    ARM_STATE_IDLE,               // 空闲
    ARM_STATE_WAIT_BTN1,          // 等待按钮1触发
    ARM_STATE_ROTATE_90,          // 机械臂旋转90°
    ARM_STATE_VISION_ADJUST,      // 视觉定位后底盘调整（由底盘线程完成）
    ARM_STATE_LIFT_TO_TARGET,     // 机械臂Z轴升降到目标位置
    ARM_STATE_PUMP_GRAB,          // 开启气泵+前移吸箱子
    ARM_STATE_WAIT_BTN2,          // 等待按钮2触发（确认吸箱成功）
    ARM_STATE_ROTATE_PLACE,       // 机械臂旋转到放箱位
    ARM_STATE_PUMP_RELEASE,       // 关闭气泵+放箱子
    ARM_STATE_MOVE_BACK,          // 放箱区网格后移
    ARM_STATE_ARM_RESET           // 机械臂归位
} Arm_BoxStateTypeDef;
Arm_BoxStateTypeDef arm_box_state = ARM_STATE_IDLE;

// 视觉定位数据（由摄像头线程更新）
typedef struct {
    float x;    // X轴偏差
    float y;    // Y轴偏差
    float z;    // Z轴目标高度
    float yaw;  // 偏航角偏差
    uint8_t valid; // 定位数据是否有效
} Vision_PosTypeDef;
Vision_PosTypeDef vision_pos = {0};


 * @brief 机械臂旋转到指定角度（复用之前的位置控制）
 * @param angle 目标角度(rad)
 
void Arm_RotateTo(float angle) {
    Motor_Pos_Control(&pos_rotate_motor, &rotate_motor, 
                      angle, ROTATE_TOLERANCE, "旋转电机");
}


 * @brief 机械臂Z轴升降到目标高度
 * @param z_target Z轴目标角度(rad，映射为高度)
 
void Arm_LiftToZ(float z_target) {
    Motor_Pos_Control(&pos_raiseandlower_motor, &raiseandlower_motor, 
                      z_target, LIFT_TOLERANCE, "升降电机");
}


 * @brief 机械臂前移（抓取电机控制）
 
void Arm_MoveForward() {
    Motor_Pos_Control(&pos_catch_motor, &catch_motor, 
                      CATCH_FORWARD_ANGLE, CATCH_TOLERANCE, "抓取电机");
}


 * @brief 机械臂后移（复位）
 
void Arm_MoveBackward() {
    Motor_Pos_Control(&pos_catch_motor, &catch_motor, 
                      CATCH_BACKWARD_ANGLE, CATCH_TOLERANCE, "抓取电机");
}



 * @brief 卷轴抓取-放箱流程线程
 
void Arm_BoxGrab_Process(void *argument) {
    osDelay(1000);
    printf("卷轴抓取流程线程启动\r\n");

    while (1) {
        switch (arm_box_state) {
            // 空闲状态：等待按钮1触发
            case ARM_STATE_IDLE:
                arm_box_state = ARM_STATE_WAIT_BTN1;
                break;

            // 等待按钮1按下（启动抓取）
            case ARM_STATE_WAIT_BTN1:
                if (HAL_GPIO_ReadPin(BTN1_GPIO_PORT, BTN1_GPIO_PIN) == GPIO_PIN_RESET) {
                    osDelay(50); // 消抖
                    if (HAL_GPIO_ReadPin(BTN1_GPIO_PORT, BTN1_GPIO_PIN) == GPIO_PIN_RESET) {
                        printf("按钮1触发：启动抓取流程\r\n");
                        arm_box_state = ARM_STATE_ROTATE_90;
                    }
                }
                osDelay(10);
                break;

            // 步骤1：机械臂旋转90°
            case ARM_STATE_ROTATE_90:
                printf("机械臂旋转90°\r\n");
                Arm_RotateTo(M_PI_2); // π/2 rad = 90°
                arm_box_state = ARM_STATE_VISION_ADJUST;
                break;

            // 步骤2：等待视觉定位+底盘调整（由底盘线程完成）
            case ARM_STATE_VISION_ADJUST:
                if (vision_pos.valid) { // 视觉数据有效
                    printf("视觉定位完成：x=%.2f,y=%.2f,z=%.2f,yaw=%.2f\r\n",
                           vision_pos.x, vision_pos.y, vision_pos.z, vision_pos.yaw);
                    arm_box_state = ARM_STATE_LIFT_TO_TARGET;
                } else {
                    printf("等待视觉定位...\r\n");
                    osDelay(200);
                }
                break;

            // 步骤3：机械臂Z轴升降到目标位置
            case ARM_STATE_LIFT_TO_TARGET:
                printf("机械臂Z轴升降到目标高度：%.2f\r\n", vision_pos.z);
                Arm_LiftToZ(vision_pos.z);
                arm_box_state = ARM_STATE_PUMP_GRAB;
                break;

            // 步骤4：开启气泵+前移吸箱子
            case ARM_STATE_PUMP_GRAB:
                printf("开启气泵+机械臂前移吸箱子\r\n");
                PUMP_ON();       // 打开气泵
                Arm_MoveForward();// 机械臂前移
                arm_box_state = ARM_STATE_WAIT_BTN2;
                break;

            // 步骤5：等待按钮2按下（操作手确认吸箱成功）
            case ARM_STATE_WAIT_BTN2:
                if (HAL_GPIO_ReadPin(BTN2_GPIO_PORT, BTN2_GPIO_PIN) == GPIO_PIN_RESET) {
                    osDelay(50); // 消抖
                    if (HAL_GPIO_ReadPin(BTN2_GPIO_PORT, BTN2_GPIO_PIN) == GPIO_PIN_RESET) {
                        printf("按钮2触发：启动放箱流程\r\n");
                        arm_box_state = ARM_STATE_ROTATE_PLACE;
                    }
                }
                osDelay(10);
                break;

            // 步骤6：机械臂旋转到放箱位
            case ARM_STATE_ROTATE_PLACE:
                printf("机械臂旋转到放箱位\r\n");
                Arm_RotateTo(PLACE_ROTATE_ANGLE); // 自定义放箱位角度
                arm_box_state = ARM_STATE_PUMP_RELEASE;
                break;

            // 步骤7：关闭气泵+放箱子
            case ARM_STATE_PUMP_RELEASE:
                printf("关闭气泵+释放箱子\r\n");
                PUMP_OFF();        // 关闭气泵
                Arm_MoveBackward();// 机械臂后移（松箱）
                arm_box_state = ARM_STATE_MOVE_BACK;
                break;

            // 步骤8：放箱区网格后移（由底盘线程完成）
            case ARM_STATE_MOVE_BACK:
                printf("放箱区网格后移\r\n");
                // 发送底盘后移指令（此处省略底盘通信代码）
                osDelay(1000); // 等待后移完成
                arm_box_state = ARM_STATE_ARM_RESET;
                break;

            // 步骤9：机械臂归位
            case ARM_STATE_ARM_RESET:
                printf("机械臂归位\r\n");
                Arm_LiftToZ(0.0f);    // Z轴归位
                Arm_RotateTo(0.0f);  // 旋转归位
                Arm_MoveBackward();  // 抓取电机归位
                arm_box_state = ARM_STATE_IDLE; // 回到空闲
                printf("卷轴抓取-放箱流程完成\r\n");
                break;

            default:
                arm_box_state = ARM_STATE_IDLE;
                break;
        }
        osDelay(20);
    }
}



 * @brief 视觉定位数据更新接口（由摄像头线程调用）
 * @param x X偏差
 * @param y Y偏差
 * @param z Z目标高度
 * @param yaw 偏航角偏差

void Vision_UpdatePos(float x, float y, float z, float yaw) {
    vision_pos.x = x;
    vision_pos.y = y;
    vision_pos.z = z;
    vision_pos.yaw = yaw;
    vision_pos.valid = 1; // 标记数据有效
}
*/