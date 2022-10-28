#ifndef AUDIO_H
#define AUDIO_H

#include <Adafruit_ZeroI2S.h>
#include <Adafruit_ZeroDMA.h>
#include "utility/dma.h"
#include <math.h>

/** A stereo sample of sine waves with a phase delay.
 * SineSample are created with a specific 
 */
class SineSample {
    // i2s interface
    static Adafruit_ZeroI2S i2s;
    // dma controller
    static Adafruit_ZeroDMA dma;
    // dma descriptor for transfer
    // we only need one descriptor because we will only every transfer one block of data at once
    static DmacDescriptor *dma_descriptor;

    // sampled data (left and right channel data interleaved LRLR)
    static constexpr int buf_size = 256;
    int data[buf_size];

    // sampling frequency of data (Hz)
    static constexpr int sample_freq = 44100;

    public:

    // Initialize the dma controller and channel
    static ZeroDMAstatus init_dma();

    // construct a sample at the given frequency (Hz) with the right channel shifted by the given phase_time (in s). A positive phase_time advances the right channel relative to the left.
    // volume ranges from 0 to ((1UL << 32) - 1)
    SineSample(int frequency, float phase_time, int volume);

    // start a dma transfer of the sample to the i2s controller
    ZeroDMAstatus run_dma_tranfer();
};

#endif