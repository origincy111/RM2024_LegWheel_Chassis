/**
 *******************************************************************************
 * @file      : TOF_middleware.h
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
#ifndef __TOF_MIDDLEWARE_H__
#define __TOF_MIDDLEWARE_H__

#ifdef __cplusplus
extern "C" {
#endif

    /* Includes ------------------------------------------------------------------*/
#include "bsp_uart.h"
#include "TOF_driver.h"
#include "TOF_reg.h"
    /* Exported macro ------------------------------------------------------------*/

    /* Exported constants --------------------------------------------------------*/
    /* Exported types ------------------------------------------------------------*/
    class TOF_t {
    public:
        void Init(UART_HandleTypeDef* _phuart, uint8_t slave_addr = 0x01);
        void Update();
        uint16_t GetDistance() { return distance_; }
        uint16_t GetConfidence() { return confidence_; }
        uint8_t IsValid() { return valid_flag_; }
        UartInstance* premote_instance;

    private:
        static void TOFCallback();
        void ProcessData();

        MODBUS_t modbus_;
        uint16_t distance_;
        uint16_t confidence_;
        uint8_t valid_flag_;
        uint8_t update_flag_;
    };
    /* Exported variables --------------------------------------------------------*/
    extern TOF_t tof;
    /* Exported function prototypes ----------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __FILE_H_ */
