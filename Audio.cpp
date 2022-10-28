#include "Audio.h"

Adafruit_ZeroI2S SineSample::i2s{};
Adafruit_ZeroDMA SineSample::dma{};
DmacDescriptor *SineSample::dma_descriptor = nullptr;

ZeroDMAstatus SineSample::init_dma()
{
    dma.setTrigger(I2S_DMAC_ID_TX_0);
    dma.setAction(DMA_TRIGGER_ACTON_BEAT);
    i2s.begin(I2S_32_BIT, sample_freq);
    return dma.allocate();
}

SineSample::SineSample(int frequency, float phase_time, int volume)
{
    int *sample = data;
    for (int i = 0; i < buf_size / 2; i++)
    {
        float time = (float)i / (float)sample_freq;
        *sample++ = sin(2 * PI * time * (float)frequency ) * volume;
        *sample++ = sin(2 * PI * (time + phase_time) * (float)frequency ) * volume;
    }
}

ZeroDMAstatus SineSample::run_dma_tranfer() {
    // setup dma descriptor if this is the first transfer
    if(dma_descriptor == nullptr) {
        dma_descriptor = dma.addDescriptor(
            data,
#if defined(__SAMD51__)
      (void *)(&I2S->TXDATA.reg),
#else
      (void *)(&I2S->DATA[0].reg),
#endif
            buf_size,
            DMA_BEAT_SIZE_WORD,
            true,
            false
        );
    }
    // otherwise, modify existing descriptor to use this sample's data
    else {
        dma.changeDescriptor(
            dma_descriptor, 
            data, 
#if defined(__SAMD51__)
      (void *)(&I2S->TXDATA.reg),
#else
      (void *)(&I2S->DATA[0].reg),
#endif
            buf_size
        );
    }

    // TODO: We need to loop samples, otherwise we will need to be constantly transfering transfers. We 
    // We need to dynamically adjust the buf_size so that it contains a complete period of the sample. Otherwise, looping the sample will cut off part of the last period.
    dma.loop(false);
    i2s.enableTx();
    return dma.startJob();
}