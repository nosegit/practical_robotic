#include <mbed.h>
#include "pid.h"
#include "motor.h"
using namespace mbed;
using namespace rtos;

#define MOTORDIRA       P0_4
#define MOTORDIRB       P0_5
#define MOTORPWMA       P0_27
#define MOTORPWMB       P1_2
#define MOTORENCODERA   P1_11
#define MOTORENCODERB   P1_12

MOTOR Motor(MOTORDIRA, MOTORPWMA, MOTORENCODERA, MOTORDIRB, MOTORPWMB, MOTORENCODERB);
float myfloat;

// --- Configuration ---
const float PID_INTERVAL_MS = 50.0;     // 50ms = 20Hz
const float PID_INTERVAL_SEC = 0.05;    // 50ms in seconds

// --- RTOS Objects (The "Alarm Clock" and "Worker") ---
Ticker pid_ticker;
Thread pid_thread(osPriorityHigh);

// --- PID Controller Instance ---
// We create one instance of the class for our left motor.
// Gains (Kp, Ki, Kd, dt)
// We set Kd=0 to make it a PI controller, which fixes jitter!
PIDController pidLeft(0.1,0.5, 0.001, PID_INTERVAL_SEC);
PIDController pidRight(0.1,0.5, 0.001, PID_INTERVAL_SEC);


// --- Global Variables (for communication between threads) ---
volatile float target_speed = 0.0;
volatile float measured_speed = 0.0;
volatile int motor_pwm_out = 0;
volatile float target_speed_R = 0.0;
volatile float measured_speed_R = 0.0;
volatile int motor_pwm_out_R = 0;


// --- 1. The Ticker ISR (The "Alarm Clock") ---
// This runs at exactly 20Hz. Its ONLY job is to wake up the worker thread.
void ticker_isr() {
    pid_thread.flags_set(0x01); // Signal the thread to run
}

// --- (Pseudo-functions for your hardware) ---
float get_speed_from_encoders() {
    // This is where your code to get smoothed_speed_tps would go
    // ...
    // return smoothed_speed_tps;
    return Motor.Get_speed(0); // Placeholder
}

void set_motor_pwm(int pwm) {
    // This is where you set your motor's PWM
    // analogWrite(MOTOR_PWM_PIN, pwm);
    Motor.Set_dir(0, pwm > 0 ? 0:1);
    Motor.Set_pwm(0, abs(pwm));
}
void set_motor_pwm_R(int pwm){
    // This is where you set your motor's PWM
    // analogWrite(MOTOR_PWM_PIN, pwm);
    Motor.Set_dir(1, pwm > 0 ? 0:1);
    Motor.Set_pwm(1, abs(pwm));
}
// --- 2. The PID Thread (The "Worker") ---
// This thread sleeps until the Ticker signals it.
void pid_thread_main() {
    while (true) {
        // Go to sleep and wait for the signal (flag 0x01) from the Ticker
        ThisThread::flags_wait_all(0x01);

        // --- Woke up! Time to run the control loop ---

        // 1. Get current speed
        // measured_speed = get_speed_from_encoders();
        measured_speed = Motor.Get_speed(0);
        measured_speed_R = Motor.Get_speed(1);


        // 2. Get target speed (this would come from the RPi)
        target_speed = myfloat; // Hardcoded target for this example

        // 3. Compute the PID output using the class method
        float pwm = pidLeft.compute(target_speed, measured_speed);
        float pwm_R = pidRight.compute(target_speed, measured_speed_R);


        // 4. Apply the output to the motor
        set_motor_pwm((int)pwm);
        set_motor_pwm_R((int)pwm_R);


        motor_pwm_out = pwm;
        motor_pwm_out_R = pwm_R;
        
    }
}

// --- 3. The Main Setup ---
void setup() {
  while(!Serial);
    Serial.begin(115200);
    Serial.print("--- PID Class with Ticker + Thread Example ---\n");

    Motor.Begin();    

    // Configure the PID controller's output for our PWM (0-255)
    pidLeft.setOutputLimits(-100, 100);

    // Start the worker thread
    pid_thread.start(pid_thread_main);

    // Start the alarm clock (Ticker) to run at 50,000us (20Hz)
    pid_ticker.attach_us(&ticker_isr, PID_INTERVAL_MS * 1000);
}

// --- 4. The Main Loop (Low Priority) ---
// The main loop() is now free for low-priority tasks, like printing.
void loop() {
    Serial.print("Target:");
    Serial.println(target_speed);
    Serial.print("Measured:");
    Serial.println(measured_speed);
    Serial.print("PWM:");
    Serial.println(motor_pwm_out);

    serialfloatin();



           
    ThisThread::sleep_for(100ms); // Print 10 times per second
}


void serialfloatin(){
  if (Serial.available() > 0) {
    
  // Read all characters until a newline ('\n') is received
  String serialDataIn = Serial.readStringUntil('\n');

  // Convert the received String to a float
  myfloat = serialDataIn.toFloat();

  // Print the float back to confirm it was stored correctly
  Serial.print("You entered: ");
  Serial.println(myfloat);
  }
}