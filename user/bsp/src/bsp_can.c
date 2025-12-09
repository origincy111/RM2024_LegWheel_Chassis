/**
 *******************************************************************************
 * @file      : bsp_can.c
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2023 Reborn Team, USTB.
 *  All Rights Reserved.
 *******************************************************************************
 */
 /* Includes ------------------------------------------------------------------*/
#include "bsp_can.h"
#include "memory.h"
#include "stdlib.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

static int idx;
static CanInstance* pcan_instance[16] = { NULL };

/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
 * @brief 使用指定的初始化配置注册 CAN 实例(并非注册，仅仅传参)
 *
 * 此函数用于使用提供的初始化配置注册 CAN 实例
 *
 * @param _pconf 指向包含初始化配置的 CanInitConf 结构的指针
 * @return 指向表示已注册 CAN 实例的 CanInstance 结构的指针
 */
CanInstance* pCanRegister(CanInitConf* _pconf) {
    //不合规的注册请求会卡在死循环
    if (idx >= 16) {
        while (1) {
        }
    }

    for (uint8_t i = 0; i < 16; i++) {
        if (pcan_instance[i]->rx_id == _pconf->rx_id && pcan_instance[i]->hcan == _pconf->hcan) {
            while (1) {
            };
        }
    }

    CanInstance* p_instance = (CanInstance*)malloc(sizeof(CanInstance));
    memset(p_instance, 0, sizeof(CanInstance));

    p_instance->tx_conf.StdId = _pconf->tx_id;
    p_instance->tx_conf.IDE = CAN_ID_STD;
    p_instance->tx_conf.RTR = CAN_RTR_DATA;
    p_instance->tx_conf.DLC = 0x08;

    p_instance->hcan = _pconf->hcan;
    p_instance->rx_id = _pconf->rx_id;
    p_instance->pCanCallBack = _pconf->pCanCallBack;

    pcan_instance[idx++] = p_instance;

    return p_instance;
}

/**
 * @brief 发送CAN消息
 *
 * 此函数使用指定的 CAN 实例发送 CAN 消息并传输缓冲区
 *
 * @param _pinstance 指向 CAN 实例的指针
 * @param _ptx_buff 指向传输缓冲区的指针
 * @return HAL_StatusTypeDef 状态值
 */
HAL_StatusTypeDef CanSend(CanInstance* _pinstance, uint8_t* _ptx_buff) {
    if (_pinstance == NULL || _pinstance->hcan->Instance == NULL) {
        return HAL_ERROR;
    }
    //尝试3个空发送邮箱,失败后返回错误消息
    if (HAL_CAN_AddTxMessage(_pinstance->hcan, &_pinstance->tx_conf, _ptx_buff, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) {
        if (HAL_CAN_AddTxMessage(_pinstance->hcan, &_pinstance->tx_conf, _ptx_buff, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK) {
            if (HAL_CAN_AddTxMessage(_pinstance->hcan, &_pinstance->tx_conf, _ptx_buff, (uint32_t*)CAN_TX_MAILBOX2) != HAL_OK) {
                return HAL_ERROR;
            }
        }
    }
    return HAL_OK;
}

/**
 * @brief 设置CAN数据长度和RTR标志位
 *
 * @param _pinstance CAN实例指针
 * @param _length 数据长度
 * @param _rtr RTR标志位
 */
void CanSetDlcAndRtr(CanInstance* _pinstance, uint8_t _length, uint8_t _rtr) {
    _pinstance->tx_conf.DLC = _length;
    _pinstance->tx_conf.RTR = _rtr;
}

/**
 * @brief CAN接收中断的回调函数
 *
 * 当CAN接收中断发生时，调用此函数
 * @param _phcan 指向CAN结构体的指针
 * @param _Fifo 与中断相关的FIFO编号
 */
static void CanRxCallBack(CAN_HandleTypeDef* _phcan, uint32_t _Fifo) {
    static CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_buff[8];
    //while (HAL_CAN_GetRxFifoFillLevel(_phcan, _Fifo)) {
    HAL_CAN_GetRxMessage(_phcan, _Fifo, &rx_header, rx_buff);
    if (rx_header.StdId == 0x115) {
        volatile int16_t a = 2;
    }
    for (uint8_t i = 0; i < 16; i++) {
            if (rx_header.StdId == pcan_instance[i]->rx_id && _phcan == pcan_instance[i]->hcan) {
                //使用自定义回调函数
                if (pcan_instance[i]->pCanCallBack != NULL) {
                    pcan_instance[i]->rx_rtr = rx_header.RTR;
                    pcan_instance[i]->rx_len = rx_header.DLC;
                    memcpy(pcan_instance[i]->rx_buff, rx_buff, rx_header.DLC);
                    pcan_instance[i]->pCanCallBack();
                }
            }
        }
        
    //}
}

/**
 * @brief 当RxFIFO0中的消息处于挂起状态时，会调用回调函数
 * @param hcan 指向包含指定CAN的配置信息的 CAN_HandleTypeDef 结构的指针。
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    CanRxCallBack(hcan, CAN_RX_FIFO0);
}