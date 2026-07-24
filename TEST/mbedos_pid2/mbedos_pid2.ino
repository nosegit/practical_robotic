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
float g_velo,g_omega;

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
        // target_speed = myfloat; // Hardcoded target for this example
        setRobotSpeed(g_velo,g_omega);

        // 3. Compute the PID output using the class method
        float pwm = pidLeft.compute(target_speed, measured_speed);
        float pwm_R = pidRight.compute(target_speed_R, measured_speed_R);


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

    processSerialCommands();

           
    ThisThread::sleep_for(100ms); // Print 10 times per second
}

#define WHEEL_RADIUS 0.024     // (meters) e.g., 2.4cm
#define WHEEL_SEPARATION 0.20  // (meters) e.g., 20cm, total dist between wheels
#define TICKS_PER_REVOLUTION 1788 // 298x6 Ticks for one full wheel rotation

// Pre-calculate the wheel circumference for efficiency
#define WHEEL_CIRCUMFERENCE 0.1508 // (2.0 * PI * WHEEL_RADIUS)

void setRobotSpeed(float v, float omega) {
    
    // --- Step 1: Inverse Kinematics ---
    // Calculate the required linear speed (m/s) for each wheel
    // L = WHEEL_SEPARATION
    float v_L = v - (omega * WHEEL_SEPARATION) / 2.0;
    float v_R = v + (omega * WHEEL_SEPARATION) / 2.0;

    // --- Step 2: Unit Conversion ---
    // Convert the linear speeds (m/s) into the PID's
    // target unit (ticks/sec)
    
    // Ticks/sec = (Revolutions/sec) * (Ticks/Revolution)
    // Revolutions/sec = (m/s) / (m/Revolution, i.e., circumference)
    
    float target_ticks_sec_L = (v_L / WHEEL_CIRCUMFERENCE) * TICKS_PER_REVOLUTION;
    float target_ticks_sec_R = (v_R / WHEEL_CIRCUMFERENCE) * TICKS_PER_REVOLUTION;

    // --- Step 3: Update Global Setpoints Safely ---
    // We disable interrupts briefly to ensure the PID threads
    // don't read the variables *while* we are writing them.
    
    // __disable_irq(); // Mbed OS / ARM way to stop interrupts
    
    target_speed = target_ticks_sec_L;
    target_speed_R = target_ticks_sec_R;
    
    // __enable_irq();  // Re-enable interrupts
}


// This function reads the line and splits it by commas.
void processSerialCommands() {
  if (Serial.available() > 0) {
    
    // Read the entire line until a newline
    String line = Serial.readStringUntil('\n');

    // This is a simple trick: add a comma to the end.
    // This ensures our parsing loop finds the last command.
    // "v100,w-5" becomes "v100,w-5,"
    line += ",";

    // Loop through the line as long as there are commas
    while (line.indexOf(',') != -1) {
      
      // 1. Find the first comma
      int commaIndex = line.indexOf(',');
      
      // 2. Extract the command before the comma
      String command = line.substring(0, commaIndex);
      
      // 3. Remove that command (and the comma) from the line
      line = line.substring(commaIndex + 1);
      
      // 4. Send the single, clean command (e.g., "v100")
      //    to our helper function to be processed.
      parseCommand(command);
    }
  }
}

// This function takes a *single* command (like "v100") and executes it.
void parseCommand(String command) {
  
    // Clean up any stray whitespace
    command.trim(); 

    if( !command.length() ){
        return;
    }

    // 1. Get the prefix
    char prefix = command.charAt(0);

    float value;
    if (command.length() > 1) {
        // 2. Get the value string
        String valueString = command.substring(1);
        // 3. Convert the value string to a float
        value = valueString.toFloat();
    }

    // 4. Use a switch statement to check the prefix
    switch (prefix) {
        
        case 'x': 
            g_omega = g_velo = 0;
            Serial.println("Reset Velocity and Omege");
        break;

        case 'v':
            g_velo = value; // Store the value
            Serial.print("Updated Velocity (velo) = ");
            Serial.println(g_velo);
        break; // Exit the switch statement
        
        case 'w':
            g_omega = value; // Store the value
            Serial.print("Updated Omega (omega) = ");
            Serial.println(g_omega);
        break; // Exit the switch statement
        
        default:
            // This runs if the prefix is not 'v' or 'w'
            Serial.print("Unknown command: ");
            Serial.println(command);
        break;
    }
}