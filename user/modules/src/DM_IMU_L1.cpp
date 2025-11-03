/**
 *******************************************************************************
 * @file      : DM_IMU_L1.cpp
 * @brief     : 达妙IMU接收源文件
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
 /* Includes ------------------------------------------------------------------*/

#include "DM_IMU_L1.h"

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

DM_IMU_Recv DM_IMUInstance;

/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

static void DM_IMUCallback(void);
static int float_to_uint(float x_float, float x_min, float x_max, int bits);
static float uint_to_float(int x_int, float x_min, float x_max, int bits);

void DM_IMU_Recv::IMU_CanRegiste(CAN_HandleTypeDef* phcan, uint16_t RX_ID, uint16_t TX_ID) {
    CanInitConf conf;
    conf.hcan = phcan;
    conf.rx_id = RX_ID;
    conf.tx_id = TX_ID;
    conf.pCanCallBack = DM_IMUCallback;
    //注册can
    IMU_CAN_Instance = pCanRegister(&conf);

    //设置IMU回调更新函数
    IMU_CAN_Instance->pCanCallBack = DM_IMUCallback;
    //请求指令长度都为4
    IMU_CAN_Instance->tx_conf.DLC = 4;
    //发送ID固定位0x6FF
    IMU_CAN_Instance->tx_conf.StdId = 0x6FF;
}

void DM_IMU_Recv::AccRTS() {
    uint8_t pdata[4] = { 0 };
    pdata[0] = (uint8_t)(TX_ID & 0xff);
    pdata[1] = (uint8_t)((TX_ID >> 8) & 0xff);
    pdata[2] = 0x01;
    pdata[3] = 0xCC;

    CanSend(IMU_CAN_Instance, pdata);
}

void DM_IMU_Recv::GyroRTS() {
    uint8_t pdata[4];
    pdata[0] = TX_ID & 0xff;
    pdata[1] = (TX_ID >> 8) & 0xff;
    pdata[2] = 0x02;
    pdata[3] = 0xCC;

    CanSend(IMU_CAN_Instance, pdata);
}

void DM_IMU_Recv::EularRTS() {
    uint8_t pdata[4];
    pdata[0] = TX_ID & 0xff;
    pdata[1] = (TX_ID >> 8) & 0xff;
    pdata[2] = 0x03;
    pdata[3] = 0xCC;

    CanSend(IMU_CAN_Instance, pdata);
}

void DM_IMU_Recv::QuatRTS() {
    uint8_t pdata[4];
    pdata[0] = TX_ID & 0xff;
    pdata[1] = (TX_ID >> 8) & 0xff;
    pdata[2] = 0x04;
    pdata[3] = 0xCC;

    CanSend(IMU_CAN_Instance, pdata);
}

void DM_IMU_Recv::Update() {
    switch (IMU_CAN_Instance->rx_buff[0]) {
    case AccPackType:
        //包类型
        LastPackType = AccPackType;
        //第二位为温度
        Temp_m = IMU_CAN_Instance->rx_buff[1];
        //第三位为x轴加速度低8位，第四位为x轴加速度高8位
        temp[0] = IMU_CAN_Instance->rx_buff[3] << 8 | IMU_CAN_Instance->rx_buff[2];
        //第五位为y轴加速度低8位，第六位为y轴加速度高8位
        temp[1] = IMU_CAN_Instance->rx_buff[5] << 8 | IMU_CAN_Instance->rx_buff[4];
        //第七位为z轴加速度低8位，第八位为z轴加速度高8位
        temp[2] = IMU_CAN_Instance->rx_buff[7] << 8 | IMU_CAN_Instance->rx_buff[6];

        AccX_m = uint_to_float(temp[0], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
        AccY_m = uint_to_float(temp[1], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
        AccZ_m = uint_to_float(temp[2], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);

        break;
    case GyroPackType:
        // 包类型
        LastPackType = GyroPackType;
        // 第三位为x轴角速度低8位，第四位为x轴角速度高8位
        temp[0] = IMU_CAN_Instance->rx_buff[3] << 8 | IMU_CAN_Instance->rx_buff[2];
        // 第五位为y轴角速度低8位，第六位为y轴角速度高8位
        temp[1] = IMU_CAN_Instance->rx_buff[5] << 8 | IMU_CAN_Instance->rx_buff[4];
        // 第七位为z轴角速度低8位，第八位为z轴角速度高8位
        temp[2] = IMU_CAN_Instance->rx_buff[7] << 8 | IMU_CAN_Instance->rx_buff[6];

        GyroX_m = uint_to_float(temp[0], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
        GyroY_m = uint_to_float(temp[1], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
        GyroZ_m = uint_to_float(temp[2], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
        break;
    case EularPackType:
        // 包类型
        LastPackType = EularPackType;
        // 第三位为Pitch低8位，第四位为Pitch高8位
        temp[0] = IMU_CAN_Instance->rx_buff[3] << 8 | IMU_CAN_Instance->rx_buff[2];
        // 第五位为Yaw低8位，第六位为Yaw高8位
        temp[1] = IMU_CAN_Instance->rx_buff[5] << 8 | IMU_CAN_Instance->rx_buff[4];
        // 第七位为Roll低8位，第八位为Roll高8位
        temp[2] = IMU_CAN_Instance->rx_buff[7] << 8 | IMU_CAN_Instance->rx_buff[6];

        Pitch_m = uint_to_float(temp[0], PITCH_CAN_MIN, PITCH_CAN_MAX, 16);
        Yaw_m = uint_to_float(temp[1], YAW_CAN_MIN, YAW_CAN_MAX, 16);
        Roll_m = uint_to_float(temp[2], ROLL_CAN_MIN, ROLL_CAN_MAX, 16);

        break;
    case QuatPackType:
        // 包类型
        LastPackType = QuatPackType;

        temp[0] = IMU_CAN_Instance->rx_buff[1] << 6 | ((IMU_CAN_Instance->rx_buff[2] & 0xF8) >> 2);
        temp[1] = (IMU_CAN_Instance->rx_buff[2] & 0x03) << 12 | (IMU_CAN_Instance->rx_buff[3] << 4)
            | ((IMU_CAN_Instance->rx_buff[4] & 0xF0) >> 4);
        temp[2] = (IMU_CAN_Instance->rx_buff[4] & 0x0F) << 10 | (IMU_CAN_Instance->rx_buff[5] << 2)
            | (IMU_CAN_Instance->rx_buff[6] & 0xC0) >> 6;
        temp[3] = (IMU_CAN_Instance->rx_buff[6] & 0x3F) << 8 | IMU_CAN_Instance->rx_buff[7];

        W_m = uint_to_float(temp[0], Quaternion_MIN, Quaternion_MAX, 14);
        X_m = uint_to_float(temp[1], Quaternion_MIN, Quaternion_MAX, 14);
        Y_m = uint_to_float(temp[2], Quaternion_MIN, Quaternion_MAX, 14);
        Z_m = uint_to_float(temp[3], Quaternion_MIN, Quaternion_MAX, 14);
        break;
    default:
        break;
    }
}

static void DM_IMUCallback(void) {
    DM_IMUInstance.Update();
}

/**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
static int float_to_uint(float x_float, float x_min, float x_max, int bits) {
    /* Converts a float to an unsigned int, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}

/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
static float uint_to_float(int x_int, float x_min, float x_max, int bits) {
    /* converts unsigned int to float, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}