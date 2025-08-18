#include "filter.h"

ForceFilter_t filters[CHANNEL_NUM] = {0};

/**
 * @brief 插入新数据到滤波缓冲区
 */
void insertToBuffer(ForceFilter_t *f, float newVal)
{
    f->buffer[f->bufIndex] = newVal;
    f->bufIndex = (f->bufIndex + 1) % FILTER_WINDOW_SIZE;
}

/**
 * @brief 计算中值
 */
float getMedianValue(ForceFilter_t *f)
{
    float temp[FILTER_WINDOW_SIZE];
    // 拷贝
    for (int i = 0; i < FILTER_WINDOW_SIZE; i++) {
        temp[i] = f->buffer[i];
    }
    // 冒泡排序
    for (int i = 0; i < FILTER_WINDOW_SIZE - 1; i++) {
        for (int j = i + 1; j < FILTER_WINDOW_SIZE; j++) {
            if (temp[i] > temp[j]) {
                float t = temp[i];
                temp[i] = temp[j];
                temp[j] = t;
            }
        }
    }
    return temp[FILTER_WINDOW_SIZE / 2]; // 中值
}

/**
 * @brief 多通道拉力传感器滤波器
 * @param channel 通道号 (0~CHANNEL_NUM-1)
 * @param newVal 新采集的ADC值
 * @return 滤波后的结果
 */
float ForceSensorFilter(uint8_t channel, float newVal)
{
    if (channel >= CHANNEL_NUM) return 0; // 越界保护
    
    ForceFilter_t *f = &filters[channel];
    
    // Step 1: 中值滤波
    insertToBuffer(f, newVal);
    float medianVal = getMedianValue(f);

    // Step 2: 一阶低通滤波
    f->filteredValue = ALPHA * medianVal + (1.0f - ALPHA) * f->filteredValue;

    return f->filteredValue;
}


