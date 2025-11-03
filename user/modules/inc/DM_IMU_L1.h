/**
 *******************************************************************************
 * @file      : DM_IMU_L1.h
 * @brief     : 达妙IMU接收
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2025.10.18      origincy        none
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2025 Reborn Team, USTB.
 *  All Rights Reserved.
 *******************************************************************************
 */
 /* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __DM_IMU_L1_H__
#define __DM_IMU_L1_H__

#ifdef __cplusplus

/* Includes ------------------------------------------------------------------*/
#include "bsp_can.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

#define ACCEL_CAN_MAX           (58.8f)
#define ACCEL_CAN_MIN	        (-58.8f)
#define GYRO_CAN_MAX	        (34.88f)
#define GYRO_CAN_MIN	        (-34.88f)
#define PITCH_CAN_MAX	        (90.0f)
#define PITCH_CAN_MIN	        (-90.0f)
#define ROLL_CAN_MAX	        (180.0f)
#define ROLL_CAN_MIN	        (-180.0f)
#define YAW_CAN_MAX		        (180.0f)
#define YAW_CAN_MIN 	        (-180.0f)
#define TEMP_MIN			    (0.0f)
#define TEMP_MAX			    (60.0f)
#define Quaternion_MIN	        (-1.0f)
#define Quaternion_MAX	        (1.0f)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 包类型枚举量，用于判断收到的包是哪种类型
 *
 * @attention : 后的为显式指示的enum底层类型
 */
typedef enum : uint8_t {
    AccPackType = 1,        //加速度包
    GyroPackType,           //角速度包
    EularPackType,          //欧拉角包
    QuatPackType            //四元数包
}IMU_PackType_t;

class DM_IMU_Recv {
private:
    //根据实际修改
    CAN_HandleTypeDef* phcan = &hcan2;
    const uint16_t RX_ID = 0x13;
    uint16_t TX_ID = 0x03;

    IMU_PackType_t LastPackType;

    float AccX_m, AccY_m, AccZ_m;
    float GyroX_m, GyroY_m, GyroZ_m;
    float Pitch_m, Yaw_m, Roll_m;
    float W_m, X_m, Y_m, Z_m;
    float Temp_m;

    float temp[4];

    uint8_t pdata[4];

    CanInstance* IMU_CAN_Instance;
public:
    DM_IMU_Recv() {
        IMU_CanRegiste(phcan, RX_ID, TX_ID);
    }

    float GetAccX_m() { return AccX_m; }
    float GetAccY_m() { return AccY_m; }
    float GetAccZ_m() { return AccZ_m; }
    float GetGyroX_m() { return GyroX_m; }
    float GetGyroY_m() { return GyroY_m; }
    float GetGyroZ_m() { return GyroZ_m; }
    float GetPitch_m() { return Pitch_m; }
    float GetYaw_m() { return Yaw_m; }
    float GetRoll_m() { return Roll_m; }
    float GetW_m() { return W_m; }
    float GetX_m() { return X_m; }
    float GetY_m() { return Y_m; }
    float GetZ_m() { return Z_m; }

    void IMU_CanRegiste(CAN_HandleTypeDef* phcan, uint16_t RX_ID, uint16_t TX_ID);

    void AccRTS();      //加速度请求
    void GyroRTS();     //角速度请求
    void EularRTS();    //欧拉角请求
    void QuatRTS();     //四元数请求

    void Update();
};
/* Exported variables --------------------------------------------------------*/

extern DM_IMU_Recv DM_IMUInstance;

/* Exported function prototypes ----------------------------------------------*/


#endif

#endif /* __DJI_MOTOR_H_ */
