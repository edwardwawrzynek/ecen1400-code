#include "Audio.h"

// Pinout for I2S connections on Seeeduino Xiao:
// SD[0] (Serial data) - D8
// SCK[0] (Serial clock) - D2
// FS[0] (word select / frame sync) - D3

SineSample sample(344, 0, (1UL << 32) - 1, (1UL << 16) - 1);

void setup() {
  delay(1000);
  Serial.println("Initializing dma channel");
  ZeroDMAstatus status = SineSample::init_dma();
  sample.print_status(status);
}

void loop() {
  Serial.println("Starting dma transfer");
  ZeroDMAstatus status = sample.run_dma_tranfer();
  sample.print_status(status);
  delay(5000);
}
