/**
 *******************************************************************************
 * @file      : TOF_driver.h
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
 /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TOF_DRIVER_H__
#define __TOF_DRIVER_H__

#ifdef __cplusplus
extern "C" {
#endif

    /* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdint.h"
#include "bsp_dwt.h"
#include "crc_def.h"
/* Exported macro ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
    /* Modbus功能码定义 */
#define MODBUS_READ_REG_CMD      0x03
#define MODBUS_WRITE_REG_CMD     0x06

/* Modbus帧长度定义 */
#define MODBUS_MIN_FRAME_LENGTH  6
#define MODBUS_HEADER_LENGTH     2
#define MODBUS_CRC_LENGTH        2

    typedef struct MODBUS_t {
        UART_HandleTypeDef* huart_;      // UART句柄
        uint8_t Master_Address_;         // 主机地址
        uint8_t Slave_Address_;          // 从机地址
        uint8_t recv_flag_;              // 1完成 0未完成
        uint8_t send_flag_;              // 发送标志
        uint16_t rx_length_;             // 接收数据长度
        uint16_t tx_length_;             // 发送数据长度
        uint8_t rx_buffer_[32];          // 接收缓冲区
        uint8_t tx_buffer_[32];          // 发送缓冲区
    } MODBUS_t;
    /* Exported variables --------------------------------------------------------*/
    /* Exported function prototypes ----------------------------------------------*/
    void MODBUS_Init(MODBUS_t* modbus, UART_HandleTypeDef* huart, uint8_t master_addr, uint8_t slave_addr);
    void MODBUS_SendReadReg(MODBUS_t* modbus, uint16_t reg_addr, uint16_t reg_num);
    void MODBUS_SendWriteReg(MODBUS_t* modbus, uint16_t reg_addr, uint16_t reg_value);
    void MODBUS_ReceiveHandler(MODBUS_t* modbus);
    uint16_t MODBUS_GetData(MODBUS_t* modbus, uint16_t index);
    uint16_t MODBUS_CRC16(uint8_t* data, uint16_t length);
#ifdef __cplusplus
}
#endif

#endif /* __FILE_H_ */
