#include "tim.h"
#include "gpio.h"
/* USER CODE BEGIN 0 */

// 气泵与电磁阀控制头文件

// 气泵继电器控制引脚（泵电源）
#define PUMP_RELAY_GPIO_PORT  Relay0_GPIO_Port
#define PUMP_RELAY_GPIO_PIN   Relay0_Pin

// 电磁阀控制引脚
#define SOLENOID_VALVE_GPIO_PORT  GPIOE
#define SOLENOID_VALVE_GPIO_PIN   GPIO_PIN_8

// 设置气泵 PWM 功率（0~100%）
void Pump_SetPower(uint8_t power);

// 电磁阀控制
void Pump_ValveOn(void);   // 打开电磁阀
void Pump_ValveOff(void);  // 关闭电磁阀

// 预留：气泵电源继电器上电控制
void Pump_ValvePowerOn(void); // 继电器上电
/* USER CODE END 0 */