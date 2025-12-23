#include "tim.h"
/* USER CODE BEGIN 0 */

// Pump control header file
// 设置气泵 PWM 功率（0~100%）
void Pump_SetPower(uint8_t power);

// 电磁阀控制（PC0）
void Pump_ValveOn(void);   // 打开电磁阀
void Pump_ValveOff(void);  // 关闭电磁阀

/* USER CODE END 0 */