#include "pump_ctrl.h"

// 内部通用功率设置实现
static void Pump_SetPowerInternal(Pump_t* hpump, uint8_t power)
{
    if (hpump == NULL || hpump->htim == NULL)
        return;

    if (power > 100)
        power = 100; // 限制最大值为100%

    // 当前 PWM 周期（自动重装载值 + 1）
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(hpump->htim) + 1;

    // 默认：power 0~100% -> 占空比 0~period
    uint32_t pulse = period * power / 100;

    if (hpump->invert)
    {
        // 若为反向输出，则比较值越大，实际功率越小
        pulse = period - pulse;
    }

    __HAL_TIM_SET_COMPARE(hpump->htim, hpump->channel, pulse);
}

void Pump_Init(Pump_t* hpump, const Pump_Config_t* config)
{
    if (hpump == NULL || config == NULL)
        return;

    hpump->htim       = config->htim;
    hpump->channel    = config->channel;
    hpump->valve_port = config->valve_port;
    hpump->pump_port  = config->pump_port;
    hpump->pump_pin   = config->pump_pin;
    hpump->valve_pin  = config->valve_pin;
    hpump->invert     = config->invert;
}

void Pump_SetPower(Pump_t* hpump, uint8_t power)
{
    Pump_SetPowerInternal(hpump, power);
}

void Pump_ValveOn(Pump_t* hpump)
{
    if (hpump == NULL || hpump->valve_port == NULL)
        return;
    HAL_GPIO_WritePin(hpump->valve_port, hpump->valve_pin, GPIO_PIN_SET);
}

void Pump_ValveOff(Pump_t* hpump)
{
    if (hpump == NULL || hpump->valve_port == NULL)
        return;
    HAL_GPIO_WritePin(hpump->valve_port, hpump->valve_pin, GPIO_PIN_RESET);
}

void Pump_RelayOn(Pump_t* hpump)
{
    if (hpump == NULL || hpump->pump_port == NULL)
        return;
    HAL_GPIO_WritePin(hpump->pump_port, hpump->pump_pin, GPIO_PIN_SET);
}

void Pump_RelayOff(Pump_t* hpump)
{
    if (hpump == NULL || hpump->pump_port == NULL)
        return;
    HAL_GPIO_WritePin(hpump->pump_port, hpump->pump_pin, GPIO_PIN_RESET);
}

void Pump_Catch(Pump_t* hpump)
{
    Pump_ValveOff(hpump);   // 关闭电磁阀
    Pump_RelayOn(hpump);   // 打开继电器
}

void Pump_Release(Pump_t* hpump)
{
    Pump_ValveOn(hpump);   // 打开电磁阀
    Pump_RelayOff(hpump);  // 关闭继电器
}