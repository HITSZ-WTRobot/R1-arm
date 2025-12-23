#include "pump_ctrl.h"

void Pump_SetPower(uint8_t power)
{
    if (power > 100) power = 100; // 限制最大值为100%
    uint32_t period = htim3.Init.Period + 1;
    uint32_t pulse  = period * (100 - power) / 100;  // 反向计算

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse);
}