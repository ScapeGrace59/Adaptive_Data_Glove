#include "Wire.h"
#include <math.h>

#define ACTION_DECREASE -1
#define ACTION_NO_CHANGE 0
#define ACTION_INCREASE 1

#define NUM_SENSORS 5
#define NUM_READINGS 5
#define NUM_STATES 10
#define NUM_ACTIONS 3

// Array to hold the pin numbers for the sensors on each finger, from thumb to pinky
int fingerPins[NUM_SENSORS] = {32, 33, 25, 26, 27};

// Calibration values for min and max flex for each sensor (finger)
int CalibrationVals[2][NUM_SENSORS] = {
    {3807, 4095, 4095, 4095, 4095}, // Max values
    {3044,  512,  512,  512,  512}  // Min values
};

// Q-table for reinforcement learning to adjust min and max calibration values
float Q_table[2][NUM_SENSORS][NUM_ACTIONS]; 
float learning_rate = 0.1;  // Learning rate for Q-learning
float discount_factor = 0.9; // Discount factor for future rewards

// Arrays for storing sensor readings, index for reading, running total, and average
int readings[NUM_SENSORS][NUM_READINGS];
int readIndex[NUM_SENSORS] = {0};
int total[NUM_SENSORS] = {0};
int average[NUM_SENSORS] = {0};

int MPU_addr = 0x68; // I2C address of the MPU-6050

int cal_gyro = 0;  // Flag for gyro calibration

// Calibration values for accelerometer and gyro
float A_cal[6] = {-388.0, -535.0, 1507.0, 1.000, 1.000, 1.000}; // Offsets and scale factors
float G_off[3] = { 1.45, 0.12, -0.49}; // Gyro offsets
#define gscale ((250./32768.0)*(PI/180.0))  // Conversion factor for gyro readings

// Quaternion for Mahony filter to hold orientation data
float q[4] = {1.0, 0.0, 0.0, 0.0};

// Mahony filter parameters
float Kp = 30.0;  // Proportional feedback constant
float Ki = 0.0;   // Integral feedback constant

// Step size for optimizing PI parameters
float stepSizeP = 1.0f;
float stepSizeI = 0.01f;
float tolerance = 0.001f;

// Variables for timing the AHRS loop and printing data
unsigned long now_ms, last_ms = 0;
unsigned long print_ms = 30; // Interval for printing angles
float yaw, pitch, roll; // Variables to store calculated angles
float lastYaw, lastPitch, lastRoll;

// Timing for periodic optimization of PI parameters
unsigned long lastOptimizationTime = 0;
const unsigned long optimizationInterval = 500; // Optimization interval

void setup() {
  // Initialize finger sensor pins
  for(int i = 0; i < NUM_SENSORS; i++) {
    pinMode(fingerPins[i], INPUT);
  }

  // Initialize I2C communication and serial monitor
  Wire.begin();
  Serial.begin(115200);

  // Initialize MPU-6050 sensor
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // Power management register
  Wire.write(0);     // Wake up MPU-6050
  Wire.endTransmission(true);

  // Calibrate flex sensors
  CalibrateFlexSensors();

  // Initialize the Q-table for reinforcement learning
  initializeQTable();
}

// Main loop for AHRS and sensor data processing
void loop()
{
  static unsigned int i = 0; // Loop counter
  static float deltat = 0;  // Loop time in seconds
  static unsigned long now = 0, last = 0; // Timing variables for micros()
  static long gsum[3] = {0}; // Gyro sum for calibration
  int16_t ax, ay, az;  // Accelerometer raw data
  int16_t gx, gy, gz;  // Gyro raw data
  int16_t Tmp; // Temperature data (not used here)

  // Vectors to store scaled data
  float Axyz[3];
  float Gxyz[3];

  // Read accelerometer and gyro data from MPU-6050
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // Starting register for accelerometer data
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14); // Request 14 registers
  int t = Wire.read() << 8;
  ax = t | Wire.read(); // Read accelerometer X
  t = Wire.read() << 8;
  ay = t | Wire.read(); // Read accelerometer Y
  t = Wire.read() << 8;
  az = t | Wire.read(); // Read accelerometer Z
  t = Wire.read() << 8;
  Tmp = t | Wire.read(); // Read temperature (not used)
  t = Wire.read() << 8;
  gx = t | Wire.read(); // Read gyro X
  t = Wire.read() << 8;
  gy = t | Wire.read(); // Read gyro Y
  t = Wire.read() << 8;
  gz = t | Wire.read(); // Read gyro Z

  // Gyro calibration block (used once at startup)
  i++;
  if (cal_gyro) {
    gsum[0] += gx; gsum[1] += gy; gsum[2] += gz;
    if (i == 500) {
      cal_gyro = 0;  // Stop calibration after 500 readings
      for (char k = 0; k < 3; k++) G_off[k] = ((float) gsum[k]) / 500.0;

      Serial.print("G_Off: ");
      Serial.print(G_off[0]);
      Serial.print(", ");
      Serial.print(G_off[1]);
      Serial.print(", ");
      Serial.print(G_off[2]);
      Serial.println();
    }
  } else {
    // Normal AHRS calculations using the Mahony filter

    // Assign raw accelerometer data to Axyz vector
    Axyz[0] = (float) ax;
    Axyz[1] = (float) ay;
    Axyz[2] = (float) az;

    // Apply offsets and scale factors to accelerometer data
    for (i = 0; i < 3; i++) Axyz[i] = (Axyz[i] - A_cal[i]) * A_cal[i + 3];

    // Apply offsets and scale factors to gyro data
    Gxyz[0] = ((float) gx - G_off[0]) * gscale;
    Gxyz[1] = ((float) gy - G_off[1]) * gscale;
    Gxyz[2] = ((float) gz - G_off[2]) * gscale;

    // Update timing for AHRS loop
    now = micros();
    deltat = (now - last) * 1.0e-6; // Time since last update in seconds
    last = now;

    // Update Mahony filter with current sensor data
    Mahony_update(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], deltat);

    // Calculate Euler angles (yaw, pitch, roll) from the quaternion
    roll  = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
    pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
    yaw   = -atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - ( q[2] * q[2] + q[3] * q[3]));

    // Convert angles to degrees
    yaw   *= 180.0 / PI;
    pitch *= 180.0 / PI;
    roll *= 180.0 / PI;

    // Print sensor data and Euler angles periodically
    now_ms = millis();
    if (now_ms - last_ms >= print_ms) {
      last_ms = now_ms;

      // Print flex sensor data for each finger
      for(int i = 0; i < NUM_SENSORS; i++) {
        float data;
        data = flexSensor(fingerPins[i], i); // Read and process sensor data
        Serial.print(data);
        if(i < NUM_SENSORS - 1) {
          Serial.print(',');
        } else {
          Serial.print(';');
        }
      }

      // Print angles for yaw, pitch, and roll
      Serial.print(yaw, 0);
      Serial.print(", ");
      Serial.print(pitch, 0);
      Serial.print(", ");
      Serial.print(roll, 0);
      Serial.print(";");

      // Placeholder for other data (set to zero here)
      Serial.print(0);
      Serial.print(',');
      Serial.print(0);
      Serial.print(',');
      Serial.print(0);
      Serial.println("");

      // Periodically optimize PI controller parameters
      unsigned long currentTime = millis();
      if (currentTime - lastOptimizationTime >= optimizationInterval) {
        lastOptimizationTime = currentTime;
        optimizePIParameters();
      }
    }
  }
}

// Mahony filter update function
void Mahony_update(float ax, float ay, float az, float gx, float gy, float gz, float deltat) {
  float recipNorm;
  float vx, vy, vz;
  float ex, ey, ez;  // Error terms
  float qa, qb, qc;
  static float ix = 0.0, iy = 0.0, iz = 0.0;  // Integral feedback terms
  float tmp;

  // Compute feedback only if accelerometer measurement is valid
  tmp = ax * ax + ay * ay + az * az;
  tmp = 0.0; // Set to zero to ignore accelerometer (tested OK)

  if (tmp > 0.0) {
    // Normalize accelerometer measurements
    recipNorm = 1.0 / sqrt(tmp);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Calculate estimated direction of gravity
    vx = q[1] * q[3] - q[0] * q[2];
    vy = q[0] * q[1] + q[2] * q[3];
    vz = q[0] * q[0] - 0.5f + q[3] * q[3];

    // Error is cross product between estimated and measured gravity direction
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    // Apply integral feedback, if enabled
    if (Ki > 0.0f) {
      ix += Ki * ex * deltat;
      iy += Ki * ey * deltat;
      iz += Ki * ez * deltat;
      gx += ix;  // Apply integral feedback
      gy += iy;
      gz += iz;
    }

    // Apply proportional feedback to gyro term
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

// Function to compute cost for PI parameter optimization
float computeCost(float kp, float ki) {
  // Calculate cost based on differences in Euler angles
  float cost = pow(yaw - lastYaw, 2) + pow(pitch - lastPitch, 2) + pow(roll - lastRoll, 2);

  // Update last known angles
  lastYaw = yaw;
  lastPitch = pitch;
  lastRoll = roll;

  return cost;
}

// Function to optimize PI controller parameters using Hooke and Jeeves method
void optimizePIParameters() {
  float bestKp = Kp;
  float bestKi = Ki;
  float bestCost = computeCost(bestKp, bestKi);

  bool improvement = true;
  
  while (improvement) {
    improvement = false;

    // Explore Kp adjustment
    float newKp = bestKp + stepSizeP;
    float newCost = computeCost(newKp, bestKi);
    if (newCost < bestCost) {
      bestKp = newKp;
      bestCost = newCost;
      improvement = true;
    } else {
      newKp = bestKp - stepSizeP;
      newCost = computeCost(newKp, bestKi);
      if (newCost < bestCost) {
        bestKp = newKp;
        bestCost = newCost;
        improvement = true;
      }
    }

    // Explore Ki adjustment
    float newKi = bestKi + stepSizeI;
    newCost = computeCost(bestKp, newKi);
    if (newCost < bestCost) {
      bestKi = newKi;
      bestCost = newCost;
      improvement = true;
    } else {
      newKi = bestKi - stepSizeI;
      newCost = computeCost(bestKp, newKi);
      if (newCost < bestCost) {
        bestKi = newKi;
        bestCost = newCost;
        improvement = true;
      }
    }

    // Reduce step sizes if no improvement found
    if (!improvement) {
      stepSizeP *= 0.5;
      stepSizeI *= 0.5;
      if (stepSizeP < tolerance && stepSizeI < tolerance) {
        break;
      }
    }
  }

  // Update PI controller parameters with optimized values
  Kp = bestKp;
  Ki = bestKi;
}

// Function to read and process flex sensor data
float flexSensor(int PIN, int finger) {
  // Subtract last reading from total
  total[finger] -= readings[finger][readIndex[finger]];

  // Read new data from sensor
  readings[finger][readIndex[finger]] = analogRead(PIN);

  // Add new reading to total
  total[finger] += readings[finger][readIndex[finger]];

  // Advance to next reading index
  readIndex[finger]++;
  if (readIndex[finger] >= NUM_READINGS) {
    readIndex[finger] = 0;
  }

  // Calculate average sensor reading
  average[finger] = total[finger] / NUM_READINGS;

  // Adaptive calibration using Q-learning
  int max_state = map(average[finger], CalibrationVals[1][finger], CalibrationVals[0][finger], 0, NUM_STATES - 1);
  int min_state = map(average[finger], CalibrationVals[1][finger], CalibrationVals[0][finger], 0, NUM_STATES - 1);

  int max_action = chooseAction(max_state, 0, finger);
  int min_action = chooseAction(min_state, 1, finger);

  adjustCalibration(0, finger, max_action);
  adjustCalibration(1, finger, min_action);

  float max = CalibrationVals[0][finger];
  float min = CalibrationVals[1][finger];

  // Clamp average to within calibration bounds
  if(average[finger] < min){
    average[finger] = min;
  }
  if(average[finger] > max){
    average[finger] = max;
  }

  // Calculate angle based on the averaged sensor reading
  float angle = 100.0 - (100.0 * (average[finger] - min)) / (max - min);

  // Update Q-table with new observations
  float reward_max = getReward(average[finger], max);
  float reward_min = getReward(average[finger], min);
  int next_max_state = map(average[finger], CalibrationVals[1][finger], CalibrationVals[0][finger], 0, NUM_STATES - 1);
  int next_min_state = map(average[finger], CalibrationVals[1][finger], CalibrationVals[0][finger], 0, NUM_STATES - 1);

  updateQTable(max_state, max_action, reward_max, next_max_state, 0, finger);
  updateQTable(min_state, min_action, reward_min, next_min_state, 1, finger);

  return angle;
}

// Function to initialize Q-table
void initializeQTable() {
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < NUM_SENSORS; j++) {
      for (int k = 0; k < NUM_ACTIONS; k++) {
        Q_table[i][j][k] = 0.0; // Start with neutral values
      }
    }
  }
}

// Function to choose action based on current state and Q-table
int chooseAction(int state, int row, int sensor) {
  int action = 0;
  float max_q_value = -1000.0;
  for (int a = 0; a < NUM_ACTIONS; a++) {
    if (Q_table[row][sensor][a] > max_q_value) {
      max_q_value = Q_table[row][sensor][a];
      action = a;
    }
  }
  return action;
}

// Function to adjust calibration values based on action
void adjustCalibration(int row, int sensor, int action) {
  if (action == ACTION_DECREASE) {
    CalibrationVals[row][sensor] -= 1;
  } else if (action == ACTION_INCREASE) {
    CalibrationVals[row][sensor] += 1;
  }
  CalibrationVals[row][sensor] = constrain(CalibrationVals[row][sensor], 0, 4095);
}

// Function to calculate reward based on sensor reading and calibration value
float getReward(float sensor_value, float calibration_value) {
  return -abs(sensor_value - calibration_value); // Negative of absolute error as reward
}

// Function to update Q-table based on new observations
void updateQTable(int state, int action, float reward, int next_state, int row, int sensor) {
  float predict = Q_table[row][sensor][action];
  float target = reward + discount_factor * max(Q_table[row][sensor], NUM_ACTIONS);
  Q_table[row][sensor][action] += learning_rate * (target - predict);
}

// Helper function to find the maximum value in Q-table for next state
float max(float q_table[NUM_ACTIONS], int num_actions) {
  float max_value = q_table[0];
  for (int i = 1; i < num_actions; i++) {
    if (q_table[i] > max_value) {
      max_value = q_table[i];
    }
  }
  return max_value;
}

// Function to calibrate flex sensors at startup
void CalibrateFlexSensors(){
  int calibrationCount = 0;
  while(calibrationCount <= 1)
  {
    /* If calibrationCount == 0, then find minimum flex value (hand relaxed position)
     * If calibrationCount == 1, then find maximum flex value (hand in fist position)
     */
    if(calibrationCount == 0)
    {
      Serial.println("Please lay your hand flat");
      delay(3000);
      Serial.println("Collecting minimum flex readings");
    }
    else if(calibrationCount == 1)
    {
      Serial.println("Please close your hand into a fist");
      delay(3000);
      Serial.println("Collecting maximum flex readings");
    }

    // Collect readings for each finger
    for(int i = 0; i < 5; i++)
    {
      int readingCount = 0;
      int readingSum = 0;
      while(readingCount < 10) // Take 10 unique readings
      {
        readingSum += analogRead(fingerPins[i]);
        readingCount += 1;
        delay(200);
      }      
      CalibrationVals[calibrationCount][i] = readingSum / 10; // Record average of 10 readings
    }
    calibrationCount += 1;
  }

  // Print calibration results
  Serial.println("Calibration Complete");
  Serial.println("In the order from thumb to pinky, your offsets are:");
  for(int i = 0; i < 2; i++)
  {
    if(i == 0){ Serial.print("Minimum Offsets:\t"); }
    else{ Serial.print("Maximum Offsets:\t"); }
    for(int j = 0; j < 5; j++)
    {
      Serial.print(CalibrationVals[i][j]);
      Serial.print(", ");
    }  
    Serial.println();
  }
}
