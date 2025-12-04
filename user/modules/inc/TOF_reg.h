/**
 *******************************************************************************
 * @file      : TOF_reg.h
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
#ifndef __TOF_REG_H__
#define __TOF_REG_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

#define TOF_VERSION             0x00
#define TOF_ADDRESS             0x01
#define TOF_BAUD                0x02
#define TOF_DISTANCE            0x03
#define TOF_CONFIDENCE          0x04
#define TOF_SETCONFIDENCE       0x05
#define TOF_THRESHOLD           0x06
#define TOF_VALID_FLAG          0x07
#define TOF_OUTPUT_SET          0x08
#define TOF_LED_MODE            0x09
#define TOF_START_FLAG          0x10
#define TOF_REFRESH_RATE        0x11
#define TOF_CORRECT_START       0x12
#define TOF_CORRECT_POSITION_1  0x13
#define TOF_CORRECT_POSITION_2  0x14
#define TOF_CORRECT_FLAG        0x15
    
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __FILE_H_ */
