#include "motor.h"

#define MOTORDIRA       P0_4
#define MOTORDIRB       P0_5
#define MOTORPWMA       P0_27
#define MOTORPWMB       P1_2
#define MOTORENCODERA   P1_11
#define MOTORENCODERB   P1_12

MOTOR Motor(MOTORDIRA, MOTORPWMA, MOTORENCODERA, MOTORDIRB, MOTORPWMB, MOTORENCODERB);
float g_omega,g_velo;
float target_speed_L,target_speed_R;
void setup() {

  while(!Serial);
  Serial.begin(9600);
  // put your setup code here, to run once:
  Motor.Begin();

  Serial.println("Motor pid test");

}

void loop() {
  // put your main code here, to run repeatedly:
  processSerialCommands();
  setRobotSpeed(g_velo,g_omega);
  Motor.Set_speed(target_speed_L, target_speed_R);

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
    
    target_speed_L = target_ticks_sec_L;
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