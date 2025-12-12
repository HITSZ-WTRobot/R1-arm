/**
 * @file    pid_motor.c
 * @author  syhanjin
 * @date    2025-09-04
 */
#include "pid_motor.h"
#include <string.h>

void MotorPID_Calculate(MotorPID_t* hpid)
{
    hpid->cur_error = hpid->ref - hpid->fdb;
    hpid->output +=
        hpid->Kp * (hpid->cur_error - hpid->prev_error1) +
        hpid->Ki * hpid->cur_error +
        hpid->Kd * (hpid->cur_error - 2 * hpid->prev_error1 + hpid->prev_error2);
    if (hpid->output > hpid->abs_output_max)
        hpid->output = hpid->abs_output_max;
    if (hpid->output < -hpid->abs_output_max)
        hpid->output = -hpid->abs_output_max;

    hpid->prev_error2 = hpid->prev_error1;
    hpid->prev_error1 = hpid->cur_error;
}

void MotorPID_Init(MotorPID_t* hpid, const MotorPID_Config_t pid_config)
{
    /* reset pid */
    memset(hpid, 0, sizeof(MotorPID_t));

    /* set pid arguments */
    hpid->Kp             = pid_config.Kp;
    hpid->Ki             = pid_config.Ki;
    hpid->Kd             = pid_config.Kd;
    hpid->abs_output_max = pid_config.abs_output_max;
}
