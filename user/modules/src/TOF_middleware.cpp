/**
 *******************************************************************************
 * @file      : TOF_middleware.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2025-11-14      origincy        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2025 Reborn Team, USTB.
 *  All Rights Reserved.
 *******************************************************************************
 */

 /* Includes ------------------------------------------------------------------*/
#include "TOF_middleware.h"
#include "bsp_uart.h"
#include <string.h>

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TOF_t tof;

/* Private function prototypes -----------------------------------------------*/
/* Exported function implementations -----------------------------------------*/

/**
 * @brief 初始化TOF传感器
 * @param _phuart: UART句柄
 * @param slave_addr: 从机地址，默认为0x01
 */
void TOF_t::Init(UART_HandleTypeDef* _phuart, uint8_t slave_addr) {
    // 初始化Modbus通信
    MODBUS_Init(&modbus_, _phuart, 0x00, slave_addr);

    // 初始化变量
    distance_ = 0;
    confidence_ = 0;
    valid_flag_ = 0;
    update_flag_ = 0;

    // 注册UART实例
    UartInitConfig conf;
    conf.huart = _phuart;
    conf.rx_buffer_size = 32;
    conf.callback_function = TOFCallback;
    premote_instance = pUartRegister(&conf);


    //开机指令
    modbus_.tx_buffer_[0] = 0x01;
    modbus_.tx_buffer_[1] = 0x06;
    modbus_.tx_buffer_[2] = 0x00;
    modbus_.tx_buffer_[3] = 0x0A;
    modbus_.tx_buffer_[4] = 0x00;
    modbus_.tx_buffer_[5] = 0x01;
    modbus_.tx_buffer_[6] = 0x68;
    modbus_.tx_buffer_[7] = 0x08;
    modbus_.tx_length_ = 8;
    HAL_UART_Transmit(modbus_.huart_, modbus_.tx_buffer_, modbus_.tx_length_, 1000);
}

/**
 * @brief 更新TOF传感器数据(Request to Send)
 */
void TOF_t::TOF_RTS() {
    // 发送读取距离和置信度寄存器的命令
    // 从TOF_DISTANCE寄存器开始，读取2个寄存器
    MODBUS_SendReadReg(&modbus_, TOF_DISTANCE, 1);

    // 处理接收到的数据
    ProcessData();
}

/**
 * @brief UART回调函数（静态）
 */
void TOF_t::TOFCallback() {
    // 调用Modbus接收处理
    MODBUS_ReceiveHandler(&tof.modbus_);

    // 设置更新标志
    tof.update_flag_ = 1;
}

/**
 * @brief 处理接收到的数据
 */
void TOF_t::ProcessData() {
    // 检查是否有新数据需要处理
    if (update_flag_ && modbus_.recv_flag_) {
        // 解析距离数据（第一个寄存器）
        distance_ = MODBUS_GetData(&modbus_, 0);

        // 解析置信度数据（第二个寄存器）
        confidence_ = MODBUS_GetData(&modbus_, 1);

        // 根据置信度判断数据是否有效
        if (confidence_ > 0) {
            valid_flag_ = 1;
        }
        else {
            valid_flag_ = 0;
        }

        // 清除标志
        modbus_.recv_flag_ = 0;
        update_flag_ = 0;

        // 清空接收缓冲区
        modbus_.rx_length_ = 0;
        memset(modbus_.rx_buffer_, 0, sizeof(modbus_.rx_buffer_));
    }
}

/* Private function implementations ------------------------------------------*/
