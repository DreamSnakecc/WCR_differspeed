#ifndef __FILTER_H__
#define __FILTER_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#define FILTER_WINDOW_SIZE 5   // ��ֵ�˲����ڴ�С (����)
#define ALPHA 0.2f             // һ�׵�ͨ�˲�ϵ�� (0~1��ԽСԽƽ��)
#define CHANNEL_NUM 3          // ADCͨ����

typedef struct {
    float buffer[FILTER_WINDOW_SIZE];  // ��ֵ�˲�������
    uint8_t bufIndex;                  // ѭ������
    float filteredValue;               // �ϴ��˲��������ͨ��
} ForceFilter_t;

// ����3��ͨ�����˲���ʵ��
extern ForceFilter_t filters[CHANNEL_NUM];

void insertToBuffer(ForceFilter_t *f, float newVal);
float getMedianValue(ForceFilter_t *f);
float ForceSensorFilter(uint8_t channel, float newVal);

#ifdef __cplusplus
}
#endif

#endif /* __FILTER_H__ */