#include <math.h>
#include <Wire.h>
#include <SparkFunSX1509.h>

// If debug information should be printed on the serial port
const bool print_debug = true;

// I2C address of the sx1509
const byte SX1509_ADDRESS = 0x3E;
SX1509 io;
// Xiao pin for the sx1509 INT signal
const byte sx1509_int_pin = 1;

// Xiao pins for left and right audio channel
const byte laudio_pin = 7;
const byte raudio_pin = 8;

// period (in micros) for slowest warning beep
int base_beep_period = 1000000;

// current sensor being (or about to be) sampled
volatile int sensor_index = 0;

// if the trig pulse has been sent for the current sensor
volatile bool triggered = false;
// the time (in micros) at which the trigger pulse was sent
volatile long trigger_start = 0;
// if the rising edge of the echo pulse has been detected
volatile bool echoed = false;
// the time the rising edge of the echo pulse was detected
volatile long start_time = 0;
// the time the falling edge of the echo pulse was detected
volatile long end_time = 0;

// time (in micros) to wait for echo response before ignoring sensor and sampling next
const int sensor_cutoff = 30000;

const int num_sensors = 6;

// current sensor distance
volatile int distances[num_sensors];

// trigger pins on the Xiao
const byte trig_pins[num_sensors] = {6,  2,  9, 10, 3,  0};
// echo pins on the sx1509
const byte echo_pins[num_sensors] = {11, 13, 9, 8,  12, 14};

// interrupt handler called on rising or falling edge of echo pin
void echoISR() {
  // mark the time that the echo pin changed
  unsigned long t_micros = micros();
  if(!echoed) {
    start_time = t_micros;
    echoed = true;
  } else {
    end_time = t_micros;
  }
  // we need to read the echo pin to clear the interrupt on sx1509
  io.digitalRead(echo_pins[sensor_index]);
}

// advance sampling of the ultrasonic sensors if needed
void advance_sensor_sampling(long t_micros) {
  // since we change the sensor sampling state variables,
  // we need to disable interrupts to avoid race conditions with echoISR
  noInterrupts();
  // if we are not waiting on a sensor response, send the trigger pulse for the next sensor
  if(triggered == false && end_time == 0) {
    digitalWrite(trig_pins[sensor_index], HIGH);
    delayMicroseconds(10);
    digitalWrite(trig_pins[sensor_index], LOW);

    triggered = true;
    trigger_start = t_micros;
  } 
  // if echoISR has marked an end_time for the last pulse, record that time and advance to next sensor
  else if(echoed && end_time != 0) {
    // record time between pulse edges
    distances[sensor_index] = end_time - start_time;
    sensor_index++;

    triggered = false;
    start_time = 0;
    end_time = 0;
    echoed = false;
  } 
  // we didn't get a response from the sensor in the expected timeframe, so advance to the next sensor
  else if(triggered && t_micros - trigger_start > sensor_cutoff) {
    distances[sensor_index] = 0;
    sensor_index++;
    
    triggered = false;
    start_time = 0;
    end_time = 0;
    echoed = false;
  }

  // if we reached the end of the array, start at the first sensor
  if(sensor_index >= num_sensors) {
    sensor_index = 0;
  }
  interrupts();
}

// width between ears (m)
const float ear_distance = 0.2;
// distance to play sounds at (m)
const float sound_radius = 1.0;
// speed of sound in air (m/us)
const float sound_velocity = 0.000343;

// calculate the required phase shift (in microseconds) for a signal at angle theta (radians) from straight ahead
float binaural_phase_delay(float theta) {
  float alpha = PI/2 - theta;
  float l1 = sqrt(
    pow(sound_radius, 2.0)*pow(sin(alpha), 2.0) + 
    pow(sound_radius*cos(alpha) + ear_distance/2, 2.0)
  );
  float l2 = sqrt(
    pow(sound_radius, 2.0)*pow(sin(alpha), 2.0) + 
    pow(sound_radius*cos(alpha) + ear_distance/2, 2.0)
  );

  return (l1 - l2) / sound_velocity;
}

// write the current state (with time t_micro) of the audio signal given by period, delay_micro, and duty (all measurements in microseconds) to pin
void writeAudioSignal(int pin, int t_micro, int period, int delay_micro, int duty_period) {
  // create channel specific times in period
  int t_period = (t_micro + delay_micro) % period;
  // write pins based on position in period
  digitalWrite(pin, t_period <= duty_period);
}

// whether audio feedback should use phase delay or different frequencies on each channel to provide direction
const bool use_phase_delays = true;

// write the current state of an alert at position, with beat_period being the period of low frequency warning on one side
void runAudioPos(float pos, int beat_period, int t_micros) {
  // the period of signal on left and right channels
  int left_period, right_period, phase_shift;

  if(use_phase_delays) {
    // play sound at 280 Hz
    left_period = 3600;
    right_period = 3600;
    // map position (0-1) to physical angles of belt (-PI/4 <-> +PI/4)
    float theta = (pos - 0.5) * PI/2;
    phase_shift = binaural_phase_delay(theta);
  } else {
    // signal direction by playing a higher frequency on the channel in the direction of object

    // play between 600 - 6000 us (150 Hz - 1.6kHz)
    left_period = 600 + 6000*pos;
    right_period = 7200 - left_period;
    // signals are at different frequency, so a phase delay isn't noticable
    phase_shift = 0;
  }

  int beat = (t_micros % beat_period < (beat_period/2)) ? 1 : 0;
  int len, ren;
  if(pos > 0.65) {
    ren = beat;
    len = 0;
  } else if (pos < 0.35) {
    ren = 0;
    len = beat;
  } else {
    ren = beat;
    len = beat;
  }

  if(len) {
    writeAudioSignal(
      laudio_pin, 
      t_micros, 
      left_period, 
      phase_shift > 0 ? phase_shift : 0, 
      left_period/2
    );
  } else {
    digitalWrite(laudio_pin, LOW);
  }

  if(ren) {
    writeAudioSignal(
      raudio_pin, 
      t_micros, 
      right_period, 
      phase_shift < 0 ? -phase_shift : 0,
      right_period/2
    );
  } else {
    digitalWrite(raudio_pin, LOW);
  }
}

// if a warning is currently triggered
bool warning = false;
// position of warning
float warning_pos = 0.0;
// the time that the current warning started
unsigned long warning_start = 0;
// the period of the current warning beep
// warning position and beep period can only be adjusted after a full period of the current beep has been played
unsigned long current_beep_period = 0;

// write the appropriate signal for the current warning, 
// and change the current warning to match measured sensor values (if needed)
void run_warnings(long t_micros) {
  int closest = distances[0];
  int closest_dir = 0;
  for(int i = 0; i < num_sensors; i++) {
    if(distances[i] != 0 && (distances[i] < closest || closest == 0)) {
      closest = distances[i];
      closest_dir = i;
    }
  }

  float pos = (double)(closest_dir) / (num_sensors - 1);

  // clear the warning state if the current beep has expired
  if(warning && t_micros > (warning_start + current_beep_period)) {
    warning = false;
  }

  // run current warning
  if(warning) {
    runAudioPos(warning_pos, current_beep_period, t_micros);
  }
  // otherwise, create new warning
  else if(closest < 8800 && closest != 0) {
    warning = true;
    warning_pos = pos;
    warning_start = t_micros;
    if(closest < 3000) {
      current_beep_period = base_beep_period / 6;
    }
    else if(closest < 4500) {
      current_beep_period = base_beep_period / 3;
    }
    else if(closest < 6000) {
      current_beep_period = base_beep_period / 2;
    }
    else if(closest < 7000) {
      current_beep_period = base_beep_period;
    }
  }
}

// setup pin modes and interrupts
void setup()
{
  if(print_debug) Serial.begin(115200);
  // start i2c
  Wire.begin();
  // start sx1509 interface
  if (io.begin(SX1509_ADDRESS) == false) {
    if(print_debug) Serial.println("Cannot start SX1509 device.");
    while (1);
  }

  for(int i = 0; i < num_sensors; i++) {
    pinMode(trig_pins[i], OUTPUT);
    digitalWrite(trig_pins[i], LOW);
    // writing the trigger pins low is sometimes interpreted by the sensor as a trig pulse,
    // so we wait for any echo signal to clear before enabling interrupts on the echo pin
    delay(100);
    io.pinMode(echo_pins[i], INPUT);
    // setup the sx1509 interrupt on echo change. This interrupt pulls INT low on change.
    io.enableInterrupt(echo_pins[i], CHANGE);
  }

  delay(100);
  // trigger echoISR when the sx1509 INT pin goes low
  attachInterrupt(sx1509_int_pin, echoISR, FALLING);

  pinMode(laudio_pin, OUTPUT);
  pinMode(raudio_pin, OUTPUT);

  delay(100);
}

// print the current reported distances if sufficient time has elapsed
long last_print = 0;
void print_distances(long t_micros) {
  if(print_debug) {
    if(t_micros - last_print > 200000) {
      for(int i = 0; i < num_sensors; i++) {
        Serial.print(distances[i]);
        Serial.print(", ");
      }
      Serial.println();

      last_print = t_micros;
    }
  }
}

void loop()
{
  int t_micros = micros();
  advance_sensor_sampling(t_micros);
  run_warnings(t_micros);
  print_distances(t_micros);

  delayMicroseconds(1);
}
