#ifndef __FILTER_H__
#define __FILTER_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#define FILTER_WINDOW_SIZE 5   // 中值滤波窗口大小 (奇数)
#define ALPHA 0.2f             // 一阶低通滤波系数 (0~1，越小越平滑)
#define CHANNEL_NUM 3          // ADC通道数

typedef struct {
    float buffer[FILTER_WINDOW_SIZE];  // 中值滤波缓冲区
    uint8_t bufIndex;                  // 循环索引
    float filteredValue;               // 上次滤波结果（低通）
} ForceFilter_t;

// 定义3个通道的滤波器实例
extern ForceFilter_t filters[CHANNEL_NUM];

void insertToBuffer(ForceFilter_t *f, float newVal);
float getMedianValue(ForceFilter_t *f);
float ForceSensorFilter(uint8_t channel, float newVal);

#ifdef __cplusplus
}
#endif

#endif /* __FILTER_H__ */