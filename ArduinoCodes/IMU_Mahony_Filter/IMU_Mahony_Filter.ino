#include "Wire.h"
#include <math.h>

// I2C address of the MPU-6050
int MPU_addr = 0x68;

// Flag to determine if gyro calibration should be done
int cal_gyro = 0;  // Set to zero to use pre-defined gyro calibration offsets

// Calibration and offset values for accelerometer and gyroscope
float A_cal[6] = {-388.0, -535.0, 1507.0, 1.000, 1.000, 1.000}; // [0..2] offsets for x, y, z; [3..5] scale factors
float G_off[3] = {1.45, 0.12, -0.49}; // Pre-defined gyro offsets for x, y, z
#define gscale ((250.0 / 32768.0) * (PI / 180.0))  // Gyro scaling factor: 250 LSB per d/s to radians/s

// Quaternion for orientation representation
float q[4] = {1.0, 0.0, 0.0, 0.0};  // Initial quaternion (no rotation)

// Mahony filter parameters
float Kp = 30.0;  // Proportional feedback constant
float Ki = 0.0;   // Integral feedback constant

// Timing variables for the Attitude and Heading Reference System (AHRS)
unsigned long now_ms, last_ms = 0;  // Millis() timers
unsigned long print_ms = 30;        // Print interval for yaw, pitch, roll

// Variables to store calculated yaw, pitch, and roll angles
float yaw, pitch, roll;

void setup() {
  Wire.begin();  // Initialize I2C communication
  Serial.begin(115200);  // Begin serial communication at 115200 baud

  // Initialize the MPU-6050 sensor
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // Write to the power management register
  Wire.write(0);     // Set to zero to wake up the MPU-6050
  Wire.endTransmission(true);
}

// Main AHRS loop
void loop() {
  static unsigned int i = 0; // Loop counter
  static float deltat = 0;   // Loop time in seconds
  static unsigned long now = 0, last = 0;  // Micros() timers
  static long gsum[3] = {0};  // Sum of gyro readings for calibration

  // Raw sensor data variables
  int16_t ax, ay, az; // Accelerometer readings
  int16_t gx, gy, gz; // Gyroscope readings
  int16_t Tmp;        // Temperature reading (not used)

  // Scaled data vectors
  float Axyz[3]; // Scaled accelerometer data
  float Gxyz[3]; // Scaled gyroscope data

  // Read accelerometer and gyroscope data from the MPU-6050
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // Starting register for reading data
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14);  // Request 14 registers of data

  // Read accelerometer and gyroscope data from the registers
  int t = Wire.read() << 8;
  ax = t | Wire.read(); // Accelerometer X-axis
  t = Wire.read() << 8;
  ay = t | Wire.read(); // Accelerometer Y-axis
  t = Wire.read() << 8;
  az = t | Wire.read(); // Accelerometer Z-axis
  t = Wire.read() << 8;
  Tmp = t | Wire.read(); // Temperature (not used)
  t = Wire.read() << 8;
  gx = t | Wire.read(); // Gyroscope X-axis
  t = Wire.read() << 8;
  gy = t | Wire.read(); // Gyroscope Y-axis
  t = Wire.read() << 8;
  gz = t | Wire.read(); // Gyroscope Z-axis

  // Gyroscope calibration
  i++;
  if (cal_gyro) {
    gsum[0] += gx; gsum[1] += gy; gsum[2] += gz;
    if (i == 500) {  // After 500 readings
      cal_gyro = 0;  // Stop calibration
      for (char k = 0; k < 3; k++) G_off[k] = ((float)gsum[k]) / 500.0;
      Serial.print("G_Off: ");
      Serial.print(G_off[0]);
      Serial.print(", ");
      Serial.print(G_off[1]);
      Serial.print(", ");
      Serial.print(G_off[2]);
      Serial.println();
    }
  } else {
    // Normal AHRS calculations
    Axyz[0] = (float)ax;
    Axyz[1] = (float)ay;
    Axyz[2] = (float)az;

    // Apply offsets and scale factors to accelerometer data
    for (i = 0; i < 3; i++) Axyz[i] = (Axyz[i] - A_cal[i]) * A_cal[i + 3];

    // Apply offsets and scale factors to gyroscope data
    Gxyz[0] = ((float)gx - G_off[0]) * gscale;
    Gxyz[1] = ((float)gy - G_off[1]) * gscale;
    Gxyz[2] = ((float)gz - G_off[2]) * gscale;

    // Timing calculations for AHRS
    now = micros();
    deltat = (now - last) * 1.0e-6;  // Time since last update in seconds
    last = now;

    // Update the orientation using Mahony filter
    Mahony_update(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], deltat);

    // Calculate yaw, pitch, and roll from the quaternion
    roll = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
    pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
    yaw = -atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - (q[2] * q[2] + q[3] * q[3]));

    // Convert angles to degrees
    yaw *= 180.0 / PI;
    pitch *= 180.0 / PI;
    roll *= 180.0 / PI;

    // Print yaw, pitch, and roll at regular intervals
    now_ms = millis();
    if (now_ms - last_ms >= print_ms) {
      last_ms = now_ms;

      Serial.print("0");
      Serial.print(",");
      Serial.print("0");
      Serial.print(",");
      Serial.print("0");
      Serial.print(",");
      Serial.print("0");
      Serial.print(",");
      Serial.print("0");
      Serial.print(';');

      Serial.print(yaw, 0);
      Serial.print(", ");
      Serial.print(pitch, 0);
      Serial.print(", ");
      Serial.print(roll, 0);
      Serial.print(";");

      Serial.print(0);
      Serial.print(',');
      Serial.print(0);
      Serial.print(',');
      Serial.print(0);
      Serial.println("");
    }
  }
}

// Function to update orientation using Mahony filter
void Mahony_update(float ax, float ay, float az, float gx, float gy, float gz, float deltat) {
  float recipNorm;
  float vx, vy, vz;
  float ex, ey, ez;  // Error terms
  float qa, qb, qc;
  static float ix = 0.0, iy = 0.0, iz = 0.0;  // Integral feedback terms
  float tmp;

  // Compute feedback only if accelerometer measurement is valid
  tmp = ax * ax + ay * ay + az * az;
  tmp = 0.0; // Ignore accelerometer (simulated valid case)

  if (tmp > 0.0) {
    // Normalize accelerometer measurements
    recipNorm = 1.0 / sqrt(tmp);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity in body frame
    vx = q[1] * q[3] - q[0] * q[2];
    vy = q[0] * q[1] + q[2] * q[3];
    vz = q[0] * q[0] - 0.5f + q[3] * q[3];

    // Error between estimated and measured direction of gravity
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    // Apply integral feedback if enabled
    if (Ki > 0.0f) {
      ix += Ki * ex * deltat;
      iy += Ki * ey * deltat;
      iz += Ki * ez * deltat;
      gx += ix;
      gy += iy;
      gz += iz;
    }

    // Apply proportional feedback
    gx += Kp * ex;
    gy += Kp * ey;
    gz += Kp * ez;
  }

  // Integrate rate of change of quaternion
  deltat = 0.5 * deltat;
  gx *= deltat;
  gy *= deltat;
  gz *= deltat;
  qa = q[0];
  qb = q[1];
  qc = q[2];

  // Update quaternion
  q[0] += (-qb * gx - qc * gy - q[3] * gz);
  q[1] += (qa * gx + qc * gz - q[3] * gy);
  q[2] += (qa * gy - qb * gz + q[3] * gx);
  q[3] += (qa * gz + qb * gy - qc * gx);

  // Normalize quaternion
  recipNorm = 1.0 / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  q[0] *= recipNorm;
  q[1] *= recipNorm;
  q[2] *= recipNorm;
  q[3] *= recipNorm;
}
