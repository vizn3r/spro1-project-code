#include <nextion.h>
#include <usart.h>
#include <stdbool.h>
#include <math.h>
#include <avr/common.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <util/atomic.h>
#include <util/delay.h>

// Make sure that the math.h defines will work
#define _USE_MATH_DEFINES

// Redefines of the PWM counter registers
#define PD5_DUTY OCR0A
#define PD6_DUTY OCR0B

// Constants for the wheel
#define WHEEL_D (float)0.066                // Diameter in m
#define WHEEL_CIRC (float)(M_PI * WHEEL_D)  // The circumference
#define N_PULSES 64                         // n pulses for one encoder wheel rotation

// Timer constants
#define OCR1A_VAL (uint16_t)(F_CPU / 1000) - 1  // Calibrated to prescaler of 1, and to configure the timer to 1ms

/*
Project requirements:
- first meter in 21 seconds
- following 3 meters within 52 seconds
- stop after 4 meters, dont stop in between the distances

Tolerances:
- +- 1s
- +- 2cm

Set the timing and the distance before starting the car

Inidicate the state of the car on display
- current distance and distance to current section
- battery V
- if it accelerates or breaks

*/

// Running variables
float rpm_avg = 0;          // Average rpm
float rpm_avg_counter = 0;  // Counter for calculating the average
float current_rpm = 0;      // Current RPM (not precise)
float current_speed = 0;    // Current speed
float delta_speed = 0;      // For calculating acceleration
float current_accel = 0;    // Current acceleration

float current_target_dist = 0;  // Target distance for this 'run'
float current_dist = 0;         // Current distance the car has travelled
float current_target_time = 0;  // Target time for this 'run'
float current_time = 0;         // Current time the car has taken
uint64_t current_time_start = 0;// Offset from the current_time for current 'run'

uint16_t time[2] = {0, 0};  // Array for the two values of time required (so one for each 'run')
uint16_t dist[2] = {0, 0};  // Array for the two distances to be travelled required (so one for each 'run')
float req_spd = 0;          // Required speed for current 'run'
float req_rpm = 0;          // Required rpm for current 'run'

bool started = 0; // If 'run' started
bool done = 0;    // If program is done
uint8_t n_run = 0;// n of 'run'

// PID variables

float kp = 0.2;         // Proportional term
float ki = 0.000002;    // Integral term
float kd = 0.0025;      // Derivation term

float pid_err = 0;      // Error calculated from the difference between rpm_avg and the req_rpm
float pid_err_prev = 0; // Previous iteration error
float pid_int = 0;      // Integral part of the PID
float pid_der = 0;      // Derivation part of the PID
int pid_pwm = 60;       // Initial PWM
int current_pwm = 0;    // Current PWM


// Must be volatile, so the compiler does not optimize these by accident
volatile uint64_t dtime = 0;            // ms counter
volatile uint64_t dtime_now = 0;        // So the functions work with the same time
volatile uint64_t update_timer = 0;     // Timer for update
volatile uint64_t rev_counter = 0;      // Total pulse count
volatile uint64_t delta_rev_count = 0;  // Pulse count since the previous iteration


// Gets data from display and stores them into global variables
void get_time_dist(void) {
  uint8_t* buff = nx_alloc_buff();

  // Ask for data
  // First get time1
  size_t buff_size = nx_send_read(buff, "get setup.n_time1_disp.val");
  // Calculate the number from response
  time[0] = nx_check(buff, buff_size)[0];
  // Clean the buffer for further use
  nx_clean_buff(buff, buff_size);
  
  // Then get time2
  buff_size = nx_send_read(buff, "get setup.n_time2_disp.val");
  time[1] = nx_check(buff, buff_size)[0];
  nx_clean_buff(buff, buff_size);
  
  // Then get dist1
  buff_size = nx_send_read(buff, "get setup.n_dist1_disp.val");
  dist[0] = nx_check(buff, buff_size)[0];
  nx_clean_buff(buff, buff_size);

  // Lastly get dist2
  buff_size = nx_send_read(buff, "get setup.n_dist2_disp.val");
  dist[1] = nx_check(buff, buff_size)[0];
  nx_clean_buff(buff, buff_size);

  free(buff);
  buff = NULL;
}

// Read analog channel (pin)
int read_analog(uint8_t chan) {
  ADMUX = (ADMUX & 0xF8) | (chan & 0x0F);
  ADCSRA |= (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC));
  return ADC;
}

// Read the battery voltage
// Might be wrong, not tested yet
float read_vbat() {
  int adc = read_analog(0);
  int adc_vref = read_analog(0x0E);
  float vref = (1.1 * 1024.0) / (float)adc_vref;
  float vbat = ((float)adc * vref) / 1024.0;
  return vbat * 2;
}

// Update status page on the nextion display
void update_status() {
  // Send all the data to the display
  nx_send("status.n_vbat.txt=\"%.3f\"", read_vbat());
  nx_send("status.n_rpm_avg.txt=\"%.3f\"", rpm_avg);
  nx_send("status.n_rpm_targ.txt=\"%.3f\"", req_rpm);
  nx_send("status.n_speed.txt=\"%.3f\"", current_speed);
  nx_send("status.n_accel.txt=\"%.3f\"", current_accel);
  nx_send("status.n_pwm.txt=\"%d\"", current_pwm);

  nx_send("status.n_t_curr.txt=\"%.3f\"", current_time);
  nx_send("status.n_t_targ.txt=\"%.3f\"", current_target_time);
  nx_send("status.n_d_curr.txt=\"%.3f\"", current_dist);
  nx_send("status.n_d_targ.txt=\"%.3f\"", current_target_dist);
  uint8_t progress = (current_time / current_target_time) * 100;
  if (progress >= 100) 
    progress = 100;

  nx_send("status.n_progress.val=%d", progress);

  // Plotting the graphs
  nx_send("add 11,0,%d", (uint8_t)rpm_avg);
  nx_send("add 21,0,%d", (uint8_t)current_speed);
  nx_send("add 22,0,%d", (uint8_t)current_accel);

  nx_send("udelete 1024"); // Clear the nextion serial buffer to prevent overflow
}


// Function that sets up the variables to be used each 'run'
void start_run() { 
  if (n_run > 1) {
    cli();
    done = 1;
    pid_pwm = 0;
    started = 0;
    PD5_DUTY = 0;
    PD6_DUTY = 0;
    nx_send("udelete 1024");
    nx_send("page results");
    return;
  }

  current_dist = 0;
  current_accel = 0;
  current_rpm = 0;
  current_time_start = dtime;
  rpm_avg = 0;
  rpm_avg_counter = 0;
  rev_counter = 0;
  delta_rev_count = 0;

  started = 1;

  current_target_dist = (float)dist[n_run] / 100.0;
  current_target_time = (float)time[n_run];

  req_spd = current_target_dist / current_target_time;
  req_rpm = (((float)current_target_dist / (float)WHEEL_CIRC) / (float)current_target_time) * 60.0;

  req_rpm *= 48.0 / 12.0;

  n_run++;
}

// Function that runs when the 'start' button on 'setup' page is pressed
void start(uint8_t* buff, size_t buff_size) { 
  get_time_dist();
  start_run();
}

// Function to restart the avr and also the nextion
void restart(uint8_t* buff, size_t buff_size) {
  nx_send("rest");
  wdt_enable(WDTO_500MS);
  while(true);
}

// Function to regulate the PWM signal to the motor driver to reach the desired rpm, hence the required speed
void pid_regulate() {
  float interval = (float)(dtime_now - update_timer) / 1000;

  pid_err = req_rpm - rpm_avg;
  pid_int += pid_err * interval;
  pid_der = (pid_err - pid_err_prev) / interval;

  // Formula for PID
  pid_pwm += (int)((kp * pid_err) + (ki * pid_int) + (kd * pid_der));

  // Prevent overflow and set minimum value
  current_pwm = pid_pwm;
  if (current_pwm > 255)
    current_pwm = 255;
  if (current_pwm < 50)
    current_pwm = 50;
  PD5_DUTY = current_pwm;
  
  pid_err_prev = pid_err;
}

// Update running variables
void update_vars() {
  current_rpm = 60 * (rev_counter - delta_rev_count) / (float)(N_PULSES * ((dtime_now - update_timer) / 1000.0));
  rpm_avg = rpm_avg + ((current_rpm - rpm_avg) / ++rpm_avg_counter);
  
  current_speed = current_rpm * WHEEL_CIRC;
  current_accel = (current_speed - delta_speed) / (float)((dtime_now - update_timer) / 1000.0);
  current_dist = WHEEL_CIRC * (rev_counter / N_PULSES) * (1.0 / 4.0);

  delta_rev_count = rev_counter;
}

// Interrupt function that runs every time the state of PD2 changes
ISR(INT0_vect) {
  ++rev_counter;
}

// Interrupt that runs every 1ms and increases the ms counter dtime
ISR(TIMER1_COMPA_vect) {
  ++dtime;
}

int main(void) {
  // Initialization of libraries
  uart_init();
  io_redirect();
  nx_init();

  // Configuration of registers
  // PWM pins stetup
  DDRD |= (DDD1 << PD5) | (DDD1 << PD6);
  PORTD |= (1 << PD5) | (1 << PD6);

  // PWM timer/counter setup
  TCCR0A |= (1 << COM0A1) | (1 << COM0B1);
  TCCR0A |= (1 << WGM01) | (1 << WGM00);
  TCCR0B |= (1 << CS00);
  PD5_DUTY = 0;
  PD6_DUTY = 0;

  // 16 bit Timer1 used for generating the 1ms interrupt using the CRC mode
  TCCR1A = 0;
  TCCR1B |= (1 << CS10) | (1 << WGM12);
  TIMSK1 |= (1 << OCIE1A);
  OCR1A = OCR1A_VAL;
 
  // Optical encoder data pin for external interrupt setup
  DDRD &= ~(1 << PD2);
  EICRA |= (1 << ISC01);
  EIMSK |= (1 << INT0);

  // Enable the interrupts
  sei();

  // Analog pin for battery voltage
  DDRC &= ~(1 << PC0);

  // Setup the start button on the setup page to execute the start() function if released
  nx_on_release(0x06, 0x01, start);
  nx_on_release(0x05, 0x03, restart);

  // Main superloop
  while(!done) {
    if (!started) {
      nx_check(NULL, 0);
      continue;
    }

    // Update every 250ms
    if (dtime - update_timer >= 250) {
      dtime_now = dtime;
      update_vars();
      pid_regulate();
      update_timer = dtime;
    }

    // Update status constantly
    current_time = (dtime - current_time_start) / 1000.0;
    update_status();

    // Check if the 'run' has reached the targed time
    if (dtime - current_time_start >= current_target_time * 1000) {
      nx_send("udelete 1024");
      nx_send("results.n_dist%d_final.txt=\"%.3f\"", n_run, current_dist);
      nx_send("results.n_time%d_final.txt=\"%.3f\"", n_run, current_time);
      start_run();
    }
  }
  return 0;
}