#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MotorShield.h>
#include <VL53L0X.h>

VL53L0X sensor;


// MPU6050 sensor
Adafruit_MPU6050 mpu;

// Motor shield
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motor1 = AFMS.getMotor(1); // Rear-left
Adafruit_DCMotor *motor2 = AFMS.getMotor(2); // Rear-right
Adafruit_DCMotor *motor3 = AFMS.getMotor(3); // Front-right
Adafruit_DCMotor *motor4 = AFMS.getMotor(4); // Front-left
// --- Constants ---
const float G = 9.81;         // m/s² conversion
const float RC = 0.3;         // High-pass filter time constant (tweak this)
const float accX_bias = 0.0;  // Replace with your actual calibration
const float accY_bias = 0.0;
const float mass = 1; 
const float accThreshold = 2;
// --- Variables ---
float rawVx = 0.0, rawVy = 0.0;               // Before filtering
float filteredVx = 0.0, filteredVy = 0.0;     // Final high-pass filtered
float prevFilteredVx = 0.0, prevFilteredVy = 0.0;
float prevRawVx = 0.0, prevRawVy = 0.0;
float prevY = 0;                             // prev acceletration in Y direction for collision detection
float lastCollisionTime = 0;
float accMagnitude = 0 ;
float y_force = 0;
float distanceY = -2.5;
float Zangle = 0.5;

// PID gains
const float Kp = 0.3, Ki = 0.02, Kd = 0.2;

// PID variables
float estVx = 0.0, estVy = 0.0;  // Estimated velocities from accel
float integralX = 0.0, integralY = 0.0, integralWz = 0.0;
float prevErrorX = 0.0, prevErrorY = 0.0, prevErrorWz = 0.0;

unsigned long prevTime = 0;
unsigned long startTime = 0;

// Robot parameters
float r = 0.05;  // Wheel radius (meters)
float Lx = 0.06;  // Distance to wheel along X-axis
float Ly = 0.06;  // Distance to wheel along Y-axis

// Default velocities (desired state)
float desiredVx = 0, desiredVy = 0, desiredWz = 0;

// Initial actual velocity
float Vx = 0, Vy = 0, Wz = 0;

// Function to compute velocity adjustment using PID
void calculateVelocityAdjustment(float accX, float accY, float gyroZ, float dt,
                                 float desiredVx, float desiredVy, float desiredWz,
                                 float &Vx_adj, float &Vy_adj, float &Wz_adj) {
    // 1. Calculate and filter velocity
    // Recalculate alpha each loop
    float alpha = RC / (RC + dt);

    // Integrate to get velocity
    rawVx += accX * dt;
    rawVy += accY * dt;

    // High-pass filter: y[n] = α * (y[n-1] + x[n] - x[n-1])
    filteredVx = alpha * (prevFilteredVx + rawVx - prevRawVx);
    filteredVy = alpha * (prevFilteredVy + rawVy - prevRawVy);

    // Update previous values
    prevRawVx = rawVx;
    prevRawVy = rawVy;
    prevFilteredVx = filteredVx;
    prevFilteredVy = filteredVy;

    // 2. Calculate velocity error
    float errorX = desiredVx - filteredVx;
    float errorY = desiredVy - filteredVy;
    float errorWz = desiredWz - gyroZ;  // GyroZ is already angular velocity

    // 3. PID for Vx
    integralX += errorX * dt;
    float derivativeX = (errorX - prevErrorX) / dt;
    prevErrorX = errorX;
    float adjustmentX = Kp * errorX + Ki * integralX + Kd * derivativeX;

    // 4. PID for Vy
    integralY += errorY * dt;
    float derivativeY = (errorY - prevErrorY) / dt;
    integralY = constrain(integralY, -1, 1);
    prevErrorY = errorY;
    float adjustmentY = Kp * errorY + Ki * integralY + Kd * derivativeY;

    // 5. PID for Wz
    integralWz += errorWz * dt;
    float derivativeWz = (errorWz - prevErrorWz) / dt;
    prevErrorWz = errorWz;
    float adjustmentWz = Kp * errorWz + Ki * integralWz + Kd * derivativeWz;

    // 6. Output adjustments (you can clamp if needed)
    Vx_adj = adjustmentX;
    Vy_adj = adjustmentY;
    Wz_adj = adjustmentWz;
}

// Function to compute wheel speeds using inverse kinematics
void calculateWheelSpeeds(float Vx, float Vy, float Wz, float &omega_fl, float &omega_fr, float &omega_rl, float &omega_rr) {
    omega_fl = (Vy - Vx - (Lx + Ly) * Wz) / r;
    omega_fr = (Vy + Vx + (Lx + Ly) * Wz) / r;
    omega_rl = (Vy + Vx - (Lx + Ly) * Wz) / r;
    omega_rr = (Vy - Vx + (Lx + Ly) * Wz) / r;
}

void setup() {
    Serial.begin(115200);

    // Initialize motor shield and IMU
    AFMS.begin();
    if (!mpu.begin()) {
        Serial.println("Failed to initialize MPU6050!");
        while (1);
    }
    sensor.init();
  sensor.setTimeout(500);

    Serial.println("MPU6050 initialized successfully!");
    motor4->setSpeed(70);
    motor3->setSpeed(70);
    motor2->setSpeed(70);
    motor1->setSpeed(70);
    motor1->run(FORWARD);
    motor2->run(FORWARD);
    motor3->run(FORWARD);
    motor4->run(BACKWARD);
    startTime = millis();  // Save the starting time
    delay(100);

}

void loop() {
      unsigned long currentTime = millis();

      //collision detection and response
/* if (accMagnitude > accThreshold && millis() - lastCollisionTime > 1000) {
    desiredVy += accMagnitude;
    lastCollisionTime = millis();
  } */
    // Get IMU data
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);
    int distance = sensor.readRangeSingleMillimeters();
    float dt = (currentTime - prevTime) / 1000.0;
    prevTime = currentTime;

    // Read acceleration and gyro
    float accX = round (accel.acceleration.x *10)/10 - 0.5;
    float accY = -1 *(accel.acceleration.y-0.1); // round(accel.acceleration.y * 10)/10 ;
    float gyroZ = round(gyro.gyro.z * 10)/10 ;

    accMagnitude = accY - prevY;
    prevY = accY;

    // Velocity adjustment
    float vx_adj, vy_adj, wz_adj;

    // Compute velocity adjustments using PID control
    calculateVelocityAdjustment(accX, accY, gyroZ, dt, desiredVx, desiredVy, desiredWz, vx_adj, vy_adj, wz_adj);
    Vx =  desiredVy; // filteredVx + vx_adj;
    Vy = desiredVy ; //filteredVy + vy_adj;
    Wz = desiredWz; // gyroZ + wz_adj;

    // Compute wheel speeds
    float omega_fl, omega_fr, omega_rl, omega_rr;
    calculateWheelSpeeds(Vx, Vy, Wz, omega_fl, omega_fr, omega_rl, omega_rr);

    // Set motor speeds (convert omega to PWM)
    motor4->setSpeed(90);
    motor3->setSpeed(90);
    motor2->setSpeed(90);
    motor1->setSpeed(90);
  // Change motors toSTOP IF VELO90t motor direction
  motor4->run(BACKWARD);
  motor3->run(FORWARD);
  motor2->run(FORWARD);
  motor1->run(FORWARD);


/*  Serial.print((int)(omega_fl));
    Serial.print(",");
    Serial.print((int)(omega_fr));
    Serial.print(",");
    Serial.print((int)(omega_rr));
    Serial.print(",");
    Serial.println((int)(omega_rl));

    // Print debugging information
    Serial.print("Adjusted Vx: "); Serial.print(Vx);
    Serial.print(" | Adjusted Vy: "); Serial.print( Vy);
     Serial.print(" | ACCVy: "); Serial.print(filteredVy);
    Serial.print(" | Adjusted Wz: "); Serial.println(Wz);
*/
  distanceY = (-1.0375 - distance/1000)  ;
  Zangle += gyroZ * dt;
    
if (Serial.available() > 0) {
  y_force = Serial.readString().toFloat();

  float observation[6] = {0.0, (distanceY), 0.0, 0.0, Zangle, gyroZ, };
  for (int i = 0; i < 6; i++) {
    Serial.print(observation[i], 3);  // Print with 3 decimal places
    if (i < 5) {
      Serial.print(",");  // Comma between values
    }
  }

  // Print desiredVy and distanceY after the observation
  Serial.print(",");
  Serial.print(desiredVy, 3);
  Serial.print(",");
  Serial.print(rawVy, 3);
  Serial.print(",");
  Serial.println(filteredVy,3);  // Ends the line with newline
}




    delay(300);
}