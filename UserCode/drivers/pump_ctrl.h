#ifndef PUMP_CTRL_H
#define PUMP_CTRL_H

#include "tim.h"
#include "gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/* USER CODE BEGIN 0 */

// 气泵与电磁阀控制头文件

//================= 通用 Pump 句柄与配置 =================//

typedef struct
{
	TIM_HandleTypeDef* htim;        // 使用的定时器
	uint32_t           channel;     // PWM 通道
    GPIO_TypeDef*    valve_port;  // 电磁阀继电器 GPIO 端口
    GPIO_TypeDef*    pump_port;  // 气泵继电器 GPIO 端口
    uint16_t         valve_pin;   // 电磁阀 GPIO 引脚
    uint16_t         pump_pin;   // 气泵 GPIO 引脚
    uint8_t          invert;      // PWM 反向：1 反向，0 正向
} Pump_Config_t;

typedef struct
{
	TIM_HandleTypeDef* htim;
	uint32_t           channel;
    GPIO_TypeDef*    valve_port;
    GPIO_TypeDef*    pump_port;
    uint16_t         valve_pin;
    uint16_t         pump_pin;
    uint8_t          invert;
} Pump_t;

//================= 实例化接口 =================//

// 根据配置初始化一个 Pump 实例
void Pump_Init(Pump_t* hpump, const Pump_Config_t* config);

// 使用实例句柄设置气泵 PWM 功率（0~100%）
void Pump_SetPower(Pump_t* hpump, uint8_t power);

// 使用实例句柄控制电磁阀
void Pump_ValveOn(Pump_t* hpump);   // 打开电磁阀
void Pump_ValveOff(Pump_t* hpump);  // 关闭电磁阀


//气泵通电继电器设置
void Pump_RelayOn(Pump_t* hpump);   // 打开继电器
void Pump_RelayOff(Pump_t* hpump);  // 关闭继电器

/* USER CODE END 0 */

#ifdef __cplusplus
}
#endif

#endif // PUMP_CTRL_H