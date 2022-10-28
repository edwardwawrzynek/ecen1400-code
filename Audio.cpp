#include "Audio.h"

Adafruit_ZeroI2S SineSample::i2s{};
Adafruit_ZeroDMA SineSample::dma{};
DmacDescriptor *SineSample::dma_descriptor = nullptr;

ZeroDMAstatus SineSample::init_dma()
{
    dma.setTrigger(I2S_DMAC_ID_TX_0);
    dma.setAction(DMA_TRIGGER_ACTON_BEAT);
    return dma.allocate();
}

SineSample::SineSample(int frequency, float phase_time, int volume)
{
    int *sample = data;
    for (int i = 0; i < buf_size / 2; i++)
    {
        *sample++ = sin(2 * PI * i / (buf_size / 2) ) * volume;
        *sample++ = sin(2 * PI * i / (buf_size / 2) ) * volume;
    }
}