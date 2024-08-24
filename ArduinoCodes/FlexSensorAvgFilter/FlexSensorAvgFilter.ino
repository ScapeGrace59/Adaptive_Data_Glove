#include <math.h>

#define NUM_SENSORS 5      // Number of sensors (one for each finger)
#define NUM_READINGS 5     // Number of readings to average for each sensor

// Pin numbers for each sensor, in the order from thumb to pinky
int fingerPins[5] = {32, 33, 25, 26, 27};

// Calibration values for each sensor (finger)
int CalibrationVals[2][NUM_SENSORS] = {
    {3807, 4095, 4095, 4095, 4095}, // Max values
    {3044,  512,  512,  512,  512}  // Min values
};

// Arrays to store readings, running totals, and averages for each sensor
int readings[NUM_SENSORS][NUM_READINGS];  // Array to store readings for each sensor
int readIndex[NUM_SENSORS] = {0};         // Array to keep track of the current index for each sensor
int total[NUM_SENSORS] = {0};             // Array to keep running totals for each sensor
int average[NUM_SENSORS] = {0};           // Array to store averages for each sensor

void setup() {
  // Initialize each sensor pin as an input
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(fingerPins[i], INPUT);
  }

  // Begin serial communication at 115200 baud rate
  Serial.begin(115200);
  while (!Serial); // Wait for the serial port to connect (for native USB devices)

  // Calibrate flex sensors
  CalibrateFlexSensors();
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

  // Print additional zero values for padding
  Serial.print(0);
  Serial.print(",");
  Serial.print(0);
  Serial.print(",");
  Serial.print(0);
  Serial.print(';');
  Serial.print(0);
  Serial.print(',');
  Serial.print(0);
  Serial.print(',');
  Serial.print(0);
  Serial.println("");

  // Unity needs a delay to read data
  delay(30); // Delay to maintain a 30 frames per second rate
}

// Function to read the output voltage from specified pin, calculate the resistance of the flex sensor,
// determine the angle bent, and print the angle bent through the serial port
float flexSensor(int PIN, int finger) {
  // Subtract the last reading from the total for the specified finger
  total[finger] -= readings[finger][readIndex[finger]];

  // Read from the sensor
  readings[finger][readIndex[finger]] = analogRead(PIN);

  // Add the new reading to the total
  total[finger] += readings[finger][readIndex[finger]];

  // Advance to the next position in the readings array
  readIndex[finger]++;

  // Wrap around to the start of the array if we reach the end
  if (readIndex[finger] >= NUM_READINGS) {
    readIndex[finger] = 0;
  }

  // Calculate the average sensor reading for the specified finger
  average[finger] = total[finger] / NUM_READINGS;

  float max = CalibrationVals[0][finger]; // Max calibration value for the finger
  float min = CalibrationVals[1][finger]; // Min calibration value for the finger

  // Clamp the average reading to within the calibrated min/max range
  if (average[finger] < min) {
    average[finger] = min;
  }
  if (average[finger] > max) {
    average[finger] = max;
  }

  // Calculate the angle based on the averaged sensor reading
  float angle = 100.0 - (100.0 * (average[finger] - min)) / (max - min);

  return angle; // Return the calculated angle
}

// Function to calibrate flex sensors
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
    calibrationCount += 1; // Move to next calibration phase
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
    for (int j = 0; j < 5; j++) {
      Serial.print(CalibrationVals[i][j]); // Print each calibration value
      Serial.print(", ");
    }
    Serial.println();
  }
}
