#include <Arduino_LSM9DS1.h>

// --- Variables ---
float gyroBiasZ = 0;      // The average error when still
float currentAngleZ = 0;  // The accumulated angle
unsigned long prevTime = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  // --- STEP 1: Calibrate Bias (Offset) ---
  Serial.println("Calibrating Gyro Bias... KEEP STILL!");
  
  float sumZ = 0;
  int sampleCount = 1000;
  
  for (int i = 0; i < sampleCount; i++) {
    while (!IMU.gyroscopeAvailable()); // Wait for new data
    float x, y, z;
    IMU.readGyroscope(x, y, z);
    sumZ += z;
  }
  
  gyroBiasZ = sumZ / sampleCount;
  
  Serial.print("Bias Found: ");
  Serial.println(gyroBiasZ);
  Serial.println("Start rotating the robot now.");
  Serial.println("Send 'r' in Serial Monitor to reset angle to 0.");
  
  prevTime = micros();
}

void loop() {
  // Check for serial command to reset angle
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'r') {
      currentAngleZ = 0;
      Serial.println("--- Angle Reset to 0 ---");
    }
  }

  // --- STEP 2: Read and Integrate ---
  if (IMU.gyroscopeAvailable()) {
    float x, y, z;
    IMU.readGyroscope(x, y, z);

    // 1. Calculate time passed (dt) in seconds
    unsigned long currentTime = micros();
    float dt = (currentTime - prevTime) / 1000000.0;
    prevTime = currentTime;

    // 2. Remove Bias
    float correctedZ = z - gyroBiasZ;

    // 3. Noise Gate (Optional but recommended)
    // If rotation is very slow (sensor noise), ignore it.
    if (abs(correctedZ) < 0.5) { 
      correctedZ = 0;
    }

    // 4. Integrate: Angle = Angle + (Rate * Time)
    // The library returns degrees/second, so this results in Degrees.
    currentAngleZ += correctedZ * dt;

    // 5. Print Output
    // We print regularly, but not every single cycle to keep Serial readable
    static long printTimer = 0;
    if (millis() - printTimer > 100) {
      Serial.print("Target: 360.0  |  Measured: ");
      Serial.println(currentAngleZ, 2); // Print with 2 decimal places
      printTimer = millis();
    }
  }

  delay(10);
}