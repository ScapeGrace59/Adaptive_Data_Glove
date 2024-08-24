#include <math.h>

#define ACTION_DECREASE -1 // Action to decrease calibration value
#define ACTION_NO_CHANGE 0 // Action to keep calibration value unchanged
#define ACTION_INCREASE 1  // Action to increase calibration value

#define NUM_SENSORS 5      // Number of sensors (one for each finger)
#define NUM_READINGS 5     // Number of readings to average for each sensor
#define NUM_STATES 10      // Number of discrete states for Q-learning
#define NUM_ACTIONS 3      // Number of possible actions in Q-learning

// Array holding the pin numbers for each finger sensor, from thumb to pinky
int fingerPins[NUM_SENSORS] = {32, 33, 25, 26, 27};

// Calibration values for each sensor
int CalibrationVals[2][NUM_SENSORS] = {
    {3807, 4095, 4095, 4095, 4095}, // Max values for each finger sensor
    {3044,  512,  512,  512,  512}  // Min values for each finger sensor
};

// Q-table for adaptive calibration using Q-learning
float Q_table[2][NUM_SENSORS][NUM_ACTIONS];
float learning_rate = 0.1;     // Learning rate for Q-learning
float discount_factor = 0.9;   // Discount factor for future rewards

// Arrays to store sensor readings, running totals, and averages for each sensor
int readings[NUM_SENSORS][NUM_READINGS];
int readIndex[NUM_SENSORS] = {0}; // Index to keep track of current reading position
int total[NUM_SENSORS] = {0};     // Running totals for each sensor
int average[NUM_SENSORS] = {0};   // Averages for each sensor

void setup() {
  // Initialize each sensor pin as an input
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(fingerPins[i], INPUT);
  }

  // Begin serial communication at 115200 baud rate
  Serial.begin(115200);
  while (!Serial); // Wait for the serial port to connect (for native USB devices)

  // Calibrate flex sensors at startup
  CalibrateFlexSensors();

  // Initialize Q-table for reinforcement learning
  initializeQTable();
}

void loop() {
  // Loop through each sensor, read the data, and print it
  for (int i = 0; i < NUM_SENSORS; i++) {
    float data = flexSensor(fingerPins[i], i); // Read and process data from the sensor
    Serial.print(data); // Print the processed angle data

    if (i < NUM_SENSORS - 1) {
      Serial.print(','); // Print a comma if not the last sensor
    } else {
      Serial.print(';'); // Print a semicolon after the last sensor data
    }
  }

  // Print additional zero values for padding (used in Unity for synchronization)
  Serial.print("0");
  Serial.print(",");
  Serial.print("0");
  Serial.print(",");
  Serial.print("0");
  Serial.print(';');
  Serial.print("0");
  Serial.print(',');
  Serial.print("0");
  Serial.print(',');
  Serial.print("0");
  Serial.println("");

  delay(30); // 30 frames a second rate
}

// Function to read the output voltage from the specified pin,
// calculate the resistance of the flex sensor, determine the angle bent,
// and update calibration using Q-learning
float flexSensor(int PIN, int finger) {
  // Subtract the last reading from the total for the specified finger
  total[finger] -= readings[finger][readIndex[finger]];

  // Read from the sensor
  readings[finger][readIndex[finger]] = analogRead(PIN);

  // Add the new reading to the total
  total[finger] += readings[finger][readIndex[finger]];

  // Advance to the next reading index
  readIndex[finger]++;
  if (readIndex[finger] >= NUM_READINGS) {
    readIndex[finger] = 0; // Wrap around if we're at the end of the array
  }

  // Calculate the average sensor reading for the specified finger
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

  // Clamp the average reading to within the calibrated min/max range
  if (average[finger] < min) {
    average[finger] = min;
  }
  if (average[finger] > max) {
    average[finger] = max;
  }

  // Calculate the angle based on the clamped sensor reading
  float angle = 100.0 - (100.0 * (average[finger] - min)) / (max - min);

  // Update Q-table based on the observed state and actions
  float reward_max = getReward(average[finger], max);
  float reward_min = getReward(average[finger], min);
  int next_max_state = map(average[finger], CalibrationVals[1][finger], CalibrationVals[0][finger], 0, NUM_STATES - 1);
  int next_min_state = map(average[finger], CalibrationVals[1][finger], CalibrationVals[0][finger], 0, NUM_STATES - 1);

  updateQTable(max_state, max_action, reward_max, next_max_state, 0, finger);
  updateQTable(min_state, min_action, reward_min, next_min_state, 1, finger);

  return angle; // Return the calculated angle
}

// Function to initialize Q-table with neutral values
void initializeQTable() {
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < NUM_SENSORS; j++) {
      for (int k = 0; k < NUM_ACTIONS; k++) {
        Q_table[i][j][k] = 0.0; // Initialize with neutral values
      }
    }
  }
}

// Function to choose action based on current state using the Q-table
int chooseAction(int state, int row, int sensor) {
  int action = 0;
  float max_q_value = -1000.0; // Initialize with a very low value
  for (int a = 0; a < NUM_ACTIONS; a++) {
    if (Q_table[row][sensor][a] > max_q_value) {
      max_q_value = Q_table[row][sensor][a];
      action = a;
    }
  }
  return action; // Return the action with the highest Q-value
}

// Function to adjust calibration values based on the chosen action
void adjustCalibration(int row, int sensor, int action) {
  if (action == ACTION_DECREASE) {
    CalibrationVals[row][sensor] -= 1; // Decrease calibration value
  } else if (action == ACTION_INCREASE) {
    CalibrationVals[row][sensor] += 1; // Increase calibration value
  }
  // Constrain calibration values to remain within valid range
  CalibrationVals[row][sensor] = constrain(CalibrationVals[row][sensor], 0, 4095);
}

// Function to calculate reward based on sensor reading and calibration value
float getReward(float sensor_value, float calibration_value) {
  return -abs(sensor_value - calibration_value); // Negative error as reward
}

// Function to update the Q-table based on observed state, action, reward, and next state
void updateQTable(int state, int action, float reward, int next_state, int row, int sensor) {
  float predict = Q_table[row][sensor][action]; // Current Q-value
  float target = reward + discount_factor * max(Q_table[row][sensor], NUM_ACTIONS); // Target value
  // Update the Q-value using the learning rate
  Q_table[row][sensor][action] += learning_rate * (target - predict);
}

// Helper function to find the maximum value in the Q-table for the next state
float max(float q_table[NUM_ACTIONS], int num_actions) {
  float max_value = q_table[0];
  for (int i = 1; i < num_actions; i++) {
    if (q_table[i] > max_value) {
      max_value = q_table[i];
    }
  }
  return max_value; // Return the maximum Q-value
}

// Function to calibrate flex sensors at startup
void CalibrateFlexSensors() {
  int calibrationCount = 0;
  while (calibrationCount <= 1) {
    /* If calibrationCount == 0, then find minimum flex value (hand relaxed position)
     * If calibrationCount == 1, then find maximum flex value (hand in fist position)
     */
    if (calibrationCount == 0) {
      Serial.println("Please lay your hand flat");
      delay(3000); // Wait for the user to position their hand
      Serial.println("Collecting minimum flex readings");
    } else if (calibrationCount == 1) {
      Serial.println("Please close your hand into a fist");
      delay(3000); // Wait for the user to position their hand
      Serial.println("Collecting maximum flex readings");
    }

    // Collect readings for each finger
    for (int i = 0; i < 5; i++) {
      int readingCount = 0;
      int readingSum = 0;
      while (readingCount < 10) { // Take 10 unique readings for better accuracy
        readingSum += analogRead(fingerPins[i]); // Sum the readings
        readingCount += 1;
        delay(200); // Short delay between readings
      }
      // Record the average of the 10 readings as the calibration value
      CalibrationVals[calibrationCount][i] = readingSum / 10;
    }
    calibrationCount += 1; // Move to the next calibration phase
  }

  // Print calibration results
  Serial.println("Calibration Complete");
  Serial.println("In the order from thumb to pinky, your offsets are:");
  for (int i = 0; i < 2; i++) {
    if (i == 0) {
      Serial.print("Minimum Offsets:\t");
    } else {
      Serial.print("Maximum Offsets:\t");
    }
    for (int j = 0; j < NUM_SENSORS; j++) {
      Serial.print(CalibrationVals[i][j]); // Print each calibration value
      Serial.print(", ");
    }
    Serial.println();
  }
}
