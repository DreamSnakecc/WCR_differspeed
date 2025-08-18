#include "filter.h"

ForceFilter_t filters[CHANNEL_NUM] = {0};

/**
 * @brief ���������ݵ��˲�������
 */
void insertToBuffer(ForceFilter_t *f, float newVal)
{
    f->buffer[f->bufIndex] = newVal;
    f->bufIndex = (f->bufIndex + 1) % FILTER_WINDOW_SIZE;
}

/**
 * @brief ������ֵ
 */
float getMedianValue(ForceFilter_t *f)
{
    float temp[FILTER_WINDOW_SIZE];
    // ����
    for (int i = 0; i < FILTER_WINDOW_SIZE; i++) {
        temp[i] = f->buffer[i];
    }
    // ð������
    for (int i = 0; i < FILTER_WINDOW_SIZE - 1; i++) {
        for (int j = i + 1; j < FILTER_WINDOW_SIZE; j++) {
            if (temp[i] > temp[j]) {
                float t = temp[i];
                temp[i] = temp[j];
                temp[j] = t;
            }
        }
    }
    return temp[FILTER_WINDOW_SIZE / 2]; // ��ֵ
}

/**
 * @brief ��ͨ�������������˲���
 * @param channel ͨ���� (0~CHANNEL_NUM-1)
 * @param newVal �²ɼ���ADCֵ
 * @return �˲���Ľ��
 */
float ForceSensorFilter(uint8_t channel, float newVal)
{
    if (channel >= CHANNEL_NUM) return 0; // Խ�籣��
    
    ForceFilter_t *f = &filters[channel];
    
    // Step 1: ��ֵ�˲�
    insertToBuffer(f, newVal);
    float medianVal = getMedianValue(f);

    // Step 2: һ�׵�ͨ�˲�
    f->filteredValue = ALPHA * medianVal + (1.0f - ALPHA) * f->filteredValue;

    return f->filteredValue;
}


