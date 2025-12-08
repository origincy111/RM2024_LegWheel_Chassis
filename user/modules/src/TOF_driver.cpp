/**
 *******************************************************************************
 * @file      : TOF_driver.cpp
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
#include "TOF_driver.h"
#include <string.h>

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function implementations -----------------------------------------*/

/**
 * @brief 初始化Modbus通信
 * @param modbus: Modbus结构体指针
 * @param huart: UART句柄
 * @param master_addr: 主机地址
 * @param slave_addr: 从机地址
 */
void MODBUS_Init(MODBUS_t* modbus, UART_HandleTypeDef* huart, uint8_t master_addr, uint8_t slave_addr) {
    modbus->huart_ = huart;
    modbus->Master_Address_ = master_addr;
    modbus->Slave_Address_ = slave_addr;
    modbus->recv_flag_ = 0;
    modbus->rx_length_ = 0;
    modbus->tx_length_ = 0;

    // 清空缓冲区
    memset(modbus->rx_buffer_, 0, sizeof(modbus->rx_buffer_));
    memset(modbus->tx_buffer_, 0, sizeof(modbus->tx_buffer_));
}

/**
 * @brief 发送读取寄存器命令
 * @param modbus: Modbus结构体指针
 * @param reg_addr: 寄存器地址
 * @param reg_num: 寄存器数量
 */
void MODBUS_SendReadReg(MODBUS_t* modbus, uint16_t reg_addr, uint16_t reg_num) {
    uint8_t index = 0;

    // 构建Modbus RTU帧
    modbus->tx_buffer_[index++] = modbus->Slave_Address_;  // 从机地址
    modbus->tx_buffer_[index++] = MODBUS_READ_REG_CMD;     // 功能码
    modbus->tx_buffer_[index++] = (reg_addr >> 8) & 0xFF;  // 寄存器地址高字节
    modbus->tx_buffer_[index++] = reg_addr & 0xFF;         // 寄存器地址低字节
    modbus->tx_buffer_[index++] = (reg_num >> 8) & 0xFF;   // 寄存器数量高字节
    modbus->tx_buffer_[index++] = reg_num & 0xFF;          // 寄存器数量低字节

    // 计算CRC
    uint16_t crc = MODBUS_CRC16(modbus->tx_buffer_, index);
    modbus->tx_buffer_[index++] = crc & 0xFF;              // CRC低字节
    modbus->tx_buffer_[index++] = (crc >> 8) & 0xFF;       // CRC高字节

    modbus->tx_length_ = index;

    // 发送数据
    HAL_UART_Transmit(modbus->huart_, modbus->tx_buffer_, modbus->tx_length_, 1000);
}

/**
 * @brief 发送写入寄存器命令
 * @param modbus: Modbus结构体指针
 * @param reg_addr: 寄存器地址
 * @param reg_value: 寄存器值
 */
void MODBUS_SendWriteReg(MODBUS_t* modbus, uint16_t reg_addr, uint16_t reg_value) {
    uint8_t index = 0;

    // // 构建Modbus RTU帧
    // modbus->tx_buffer_[index++] = modbus->Slave_Address_;  // 从机地址
    // modbus->tx_buffer_[index++] = MODBUS_WRITE_REG_CMD;    // 功能码
    // modbus->tx_buffer_[index++] = (reg_addr >> 8) & 0xFF;  // 寄存器地址高字节
    // modbus->tx_buffer_[index++] = reg_addr & 0xFF;         // 寄存器地址低字节
    // modbus->tx_buffer_[index++] = (reg_value >> 8) & 0xFF; // 寄存器值高字节
    // modbus->tx_buffer_[index++] = reg_value & 0xFF;        // 寄存器值低字节

    // // 计算CRC
    // uint16_t crc = MODBUS_CRC16(modbus->tx_buffer_, index);
    // modbus->tx_buffer_[index++] = crc & 0xFF;              // CRC低字节
    // modbus->tx_buffer_[index++] = (crc >> 8) & 0xFF;       // CRC高字节

    // modbus->tx_length_ = index;

    // 构建Modbus RTU帧
    modbus->tx_buffer_[index++] = modbus->Slave_Address_;  // 从机地址
    modbus->tx_buffer_[index++] = MODBUS_WRITE_REG_CMD;    // 功能码
    modbus->tx_buffer_[index++] = (reg_addr >> 8) & 0xFF;  // 寄存器地址高字节
    modbus->tx_buffer_[index++] = reg_addr & 0xFF;         // 寄存器地址低字节
    modbus->tx_buffer_[index++] = (reg_value >> 8) & 0xFF; // 寄存器值高字节
    modbus->tx_buffer_[index++] = reg_value & 0xFF;        // 寄存器值低字节

    // 计算CRC
    uint16_t crc = MODBUS_CRC16(modbus->tx_buffer_, index);
    modbus->tx_buffer_[index++] = 0x74;              // CRC低字节
    modbus->tx_buffer_[index++] = 0x0A;       // CRC高字节

    modbus->tx_length_ = index;

    // 发送数据
    HAL_UART_Transmit(modbus->huart_, modbus->tx_buffer_, modbus->tx_length_, 1000);
}

/**
 * @brief 接收数据处理
 * @param modbus: Modbus结构体指针
 */
void MODBUS_ReceiveHandler(MODBUS_t* modbus) {
    // 检查是否有数据接收
    if (__HAL_UART_GET_FLAG(modbus->huart_, UART_FLAG_RXNE)) {
        uint8_t data;
        HAL_UART_Receive(modbus->huart_, &data, 1, 0);

        // 将数据存入接收缓冲区.
        if (modbus->rx_length_ < sizeof(modbus->rx_buffer_)) {
            modbus->rx_buffer_[modbus->rx_length_++] = data;
        }

        // 检查是否收到完整帧（至少6字节）
        if (modbus->rx_length_ >= MODBUS_MIN_FRAME_LENGTH) {
            // 验证CRC
            uint16_t received_crc = (modbus->rx_buffer_[modbus->rx_length_ - 1] << 8) |
                modbus->rx_buffer_[modbus->rx_length_ - 2];
            uint16_t calculated_crc = MODBUS_CRC16(modbus->rx_buffer_, modbus->rx_length_ - 2);

            if (received_crc == calculated_crc &&
                modbus->rx_buffer_[0] == modbus->Slave_Address_) {
                modbus->recv_flag_ = 1;  // 接收完成
            }
        }
    }
}

/**
 * @brief 获取接收到的数据
 * @param modbus: Modbus结构体指针
 * @param index: 数据索引
 * @return 数据值
 */
uint16_t MODBUS_GetData(MODBUS_t* modbus, uint16_t index) {
    if (modbus->recv_flag_ && index < (modbus->rx_length_ - MODBUS_HEADER_LENGTH - MODBUS_CRC_LENGTH)) {
        uint8_t data_index = MODBUS_HEADER_LENGTH + index * 2;
        return (modbus->rx_buffer_[data_index] << 8) | modbus->rx_buffer_[data_index + 1];
    }
    return 0;
}

/**
 * @brief Modbus CRC16计算
 * @param data: 数据指针
 * @param length: 数据长度
 * @return CRC16值
 */
uint16_t MODBUS_CRC16(uint8_t* data, uint16_t length) {
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else {
                crc >>= 1;
            }
        }
    }

    return crc;
}

/* Private function implementations ------------------------------------------*/
