/**
 * @file    can_driver.c
 * @author  syhanjin
 * @date    2025-09-04
 * @brief
 */
#include "bsp/can_driver.h"
#include "libs/printf.h"

/**
 * 发送一条 CAN 消息
 * @param hcan can handle
 * @param header CAN_TxHeaderTypeDef
 * @param data 数据
 * @note 本身想做成内联展开，但是必须写到 .h 文件，调研发现性能损失不大，所以直接放到此处
 * @return mailbox
 */
uint32_t CAN_SendMessage(CAN_HandleTypeDef* hcan, const CAN_TxHeaderTypeDef header, uint8_t data[])
{
    uint32_t mailbox = 0xFFFF;

    // 等待上一个发送完成
    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0)
        ;

    if (HAL_CAN_AddTxMessage(hcan, &header, data, &mailbox) != HAL_OK)
    {
        CAN_ERROR_HANDLER();
    }

    return mailbox;
}

/**
 * CAN 初始化
 * @param hcan can handle
 * @param ActiveITs CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING
 */
void CAN_Start(CAN_HandleTypeDef* hcan, const uint32_t ActiveITs)
{
    if (HAL_CAN_Start(hcan) != HAL_OK)
    {
        HAL_CAN_GetError(hcan);
        CAN_ERROR_HANDLER();
        
    }

    if (HAL_CAN_ActivateNotification(hcan, ActiveITs) != HAL_OK)
    {
        CAN_ERROR_HANDLER();
    }
}

