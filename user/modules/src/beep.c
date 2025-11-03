#include "beep.h"
#include "cmsis_os.h"
#include "tim.h"

// #define  proport          84000

// #define  L1       ((proport/73.4)-1)//低调　do 的频率
// #define  L2       ((proport/82.4)-1)//低调　re 的频率
// #define  L3       ((proport/87.3)-1)//低调　mi 的频率
// #define  L4       ((proport/98.0)-1)//低调　fa 的频率
// #define  L5       ((proport/110.0)-1)//低调　sol 的频率
// #define  L6       ((proport/123.5)-1)//低调　la 的频率
// #define  L7       ((proport/130.8)-1)//低调　si 的频率

// #define  M1       ((proport/146.8)-1)//中调　do 的频率
// #define  M2       ((proport/164.8)-1)//中调　re 的频率
// #define  M3       ((proport/174.6)-1)//中调　mi 的频率
// #define  M4       ((proport/196.0)-1)//中调　fa 的频率
// #define  M5       ((proport/220.0)-1)//中调　sol的频率
// #define  M5_      ((proport/233.1)-1)//
// #define  M6       ((proport/246.9)-1)//中调　la 的频率
// #define  M7       ((proport/261.6)-1)//中调　si 的频率

// #define  H1       ((proport/293.6)-1)//高调　do 的频率
// #define  H2       ((proport/329.6)-1)//高调　re 的频率
// #define  H3       ((proport/349.2)-1)//高调　mi 的频率
// #define  H4       ((proport/392.0)-1)//高调　fa 的频率
// #define  H5       ((proport/440.0)-1)//高调　sol的频率
// #define  H6       ((proport/493.9)-1)//高调　la 的频率
// #define  H7       ((proport/523.3)-1)//高调　si 的频率

// #define  Z0       0

// const float Music[] =
// {
//         Z0,12,Z0,12,Z0,12,Z0,12,		Z0,12,Z0,12,Z0,12,Z0,6,M1,3,M2,3,
//         M3,6,M3,6,M3,6,M3,6,			H1,9,M3,3,M3,6,M3,6,
//         M3,3,M4,3,M2,6,M2,24,			Z0,6,L7,3,M1,3,
//         M2,6,M2,6,M2,6,M2,6,			M4,4,M4,4,M4,4,
//         M4,4,M3,4,M2,4,						M1,36,Z0,6,M4,3,M5,3,
//         M6,6,M6,6,M6,6,M6,6,			M5,4,M5,4,M4,4,
//         M4,4,M3,4,M2,4,						M4,6,M3,6,M3,24,Z0,6,M4,3,M5,3,
//         M6,6,M6,6,M6,6,M6,6,			M6,4,M6,4,M6,4,
//         M6,4,M5_,4,M6,4,					M7,12,Z0,6,M3,6,

//         H1,6,H1,6,H1,6,H1,6,			H1,12,M7,6,M7,6,
//         H2,6,H2,6,H1,6,M7,6,			M7,9,H1,3,M6,6,M6,6,
//         M6,6,M6,6,M6,6,M6,6,			M5,9,H1,3,H3,6,H3,6,
//         H2,18,H3,3,H4,3,					H3,12,Z0,6,M3,6,
//         H1,6,H1,6,H1,6,H1,6,			H1,12,M7,6,M7,6,
//         H2,6,H2,6,H1,6,M7,6,			M7,9,H1,3,M6,6,M6,6,
//         M6,6,M6,6,M5,6,M4,6,			M3,3,M6,3,H1,12,M3,6,
//         M3,6,M5_,6,M7,6,M5,6,			M6,12,Z0,12,Z0,12,Z0,12

// };

// const uint8_t linkFlag[] = {
//             0x00,								0x00,
//             0x14,								0xa0,
//             0x02,								0x28,
//             0x04,								0xa8,
//             0x00,								0x80,
//             0x80,								0x10,
//             0x00,								0x10,
//             0x10,								0x00
// };

// uint16_t length = sizeof(Music) / sizeof(Music[0]);

//static void buzzer_on(uint16_t music, uint16_t volume);

void warning() {
    // uint8_t index = 0;
    // for (uint16_t i = 0;i < (length / 2);i++) {
    //     buzzer_on(Music[i * 2], 800);
    //     osDelay(66 * Music[i * 2 + 1]);
    //     if (!(linkFlag[2 * i / 8] & (0x01 << index++)))
    //         buzzer_on(Z0, 0);
    //     osDelay(50);
    //     index %= 8;
    // }
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 50);

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 50);
    osDelay(300);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
    osDelay(300);

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
}

// void buzzer_on(uint16_t music, uint16_t volume) {
//     if (music == Z0) {
//         __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
//         return;
//     }
//     __HAL_TIM_SET_PRESCALER(&htim4, music);
//     __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, volume);
// }