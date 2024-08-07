
//#include <Arduino.h>

#define PIN_flow_meter 3
#define PIN_pump_pwm 5

const int PWM_RESOLUTION = 10; // 10 bit resolution for the PWM output
const long PWM_MAX_VAL = (1 << PWM_RESOLUTION) - 1;
const float MIN_DUTY = 0.16; // 0.16 is the measured duty cycle for pump to run at lowest speed
const float MAX_DUTY = 0.95; // 0.90 is the measured duty cycle for pump to run at full speed
float curr_duty_cycle = 0;
const float pump_init_val = 25.0;

unsigned prev_time = 0;
unsigned long dT = 0;
const long MIN_dT = 2000; // minimum time between rising edge pulses that will be considered. With our flow meter, a
                          // dT of 2000 would corrispond to a flow rate of 1000 l/min, much higher than the flow meter max of 150LPM

const byte window_size = 25; // Moving average window size

static byte curr_i = 0;    // Index for current value
static byte val_count = 0; // Count of values read (<= window_size)
static float sum = 0;      // Rolling sum
static float values[window_size];

void flow_meter_pulse();
void set_pump(float duty_cycle);
void wake_pump();

void setup()
{
  Serial.begin(115200);
  pinMode(PIN_flow_meter, INPUT_PULLDOWN);
  pinMode(PIN_pump_pwm, OUTPUT);
  analogWriteRes(PWM_RESOLUTION);

  attachInterrupt(digitalPinToInterrupt(PIN_flow_meter), flow_meter_pulse, RISING);

  wake_pump();

  set_pump(pump_init_val);

  prev_time = micros();
}

void loop()
{
  if (Serial.available())
  {
    float new_duty_cycle = Serial.parseFloat(); // read new duty cycle from user as percent (0.00-100.00)

    set_pump(new_duty_cycle); // set duty cycle as a percent, to be converted to an int by set_pump()
  }
  delay(1);
}

void wake_pump()
{
  // pump needs short pulse to wake up:
  delay(10);
  digitalWrite(PIN_pump_pwm, LOW);
  delay(10);
  digitalWrite(PIN_pump_pwm, HIGH);
  delay(10);
  digitalWrite(PIN_pump_pwm, LOW);
  delay(10);
  set_pump(pump_init_val);
}

void set_pump(float duty_cycle)
{

  int duty_cycle_100 = (int)(duty_cycle * 100.0);                                  // multiply float by 100 before converting to capture 2 decimal places
  long analog_write_val_100 = map(duty_cycle_100, 0, 10000, 0, PWM_MAX_VAL * 100); // mapped ranges are 100x for same reason
  long analog_write_val = analog_write_val_100 / 100;                              // divide by 100 to return to the range accepted by analogWrite()

  if (analog_write_val >= MIN_DUTY * PWM_MAX_VAL && analog_write_val <= MAX_DUTY * PWM_MAX_VAL)
  { // check if the set value is in bounds
    analogWrite(PIN_pump_pwm, analog_write_val);
    curr_duty_cycle = duty_cycle;
  }else if(analog_write_val == 0)
  { // ignore line end characters. \r\n returns 0 from Serial.parseFloat()
    return;
  }
  else
  { 
    Serial.printf("Pump duty cycle must be between [%.2f - %.2f]\n", MIN_DUTY * 100, MAX_DUTY * 100); // allert user if set value is out of bounds
  }
}

void flow_meter_pulse()
{
  unsigned long curr_time = micros();
  long dT = curr_time - prev_time;

  // guard to catch double trigger events from bounce in the flow meter signal
  if (dT < MIN_dT)
    return;

  // beginning of dT windowed average calculation:
  sum += dT;
  // If the window is full, adjust the sum by deleting the oldest value
  if (val_count == window_size)
    sum -= values[curr_i];

  values[curr_i] = dT; // Replace the oldest with the latest

  if (++curr_i >= window_size)
    curr_i = 0;

  if (val_count < window_size)
    val_count += 1;

  // final calculation of average dT from the sum of 'window_size' previous dT measurements
  float dT_avg = sum / val_count;

  float freq = 1000000.0 / dT; // convert between signal period (dT) in microseconds to frequency in Hz
  float freq_avg = 1000000.0 / dT_avg;

  // print result. Print statement runs much faster than the flow meter can spin and no issues with printing from the ISR have been observed
  Serial.printf("Time_(s):%012.6f\tPump_percent:%.2f\tInst._flow_rate(LPM):%05.2f\tAvg._flow_rate(LPM):%05.2f\n",
            curr_time / 1000000.0, curr_duty_cycle, freq * 2, freq_avg * 2);

  // update time for next dT calculation
  prev_time = curr_time;
}
