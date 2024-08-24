#include <math.h>

#define NUM_SENSORS 5      // Number of sensors (one for each finger)
#define NUM_READINGS 5     // Number of readings to average for each sensor

// Array holding the pin numbers for each finger sensor, from thumb to pinky
int fingerPins[NUM_SENSORS] = {32, 33, 25, 26, 27};

// Calibration values for each sensor
int CalibrationVals[2][NUM_SENSORS] = {
    {3807, 4095, 4095, 4095, 4095}, // Max values for each finger sensor
    {3044,  512,  512,  512,  512}  // Min values for each finger sensor
};

void setup() {
  // Initialize each sensor pin as an input
  for(int i = 0; i < NUM_SENSORS; i++) {
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
  for(int i = 0; i < NUM_SENSORS; i++) {
    float data = flexSensor(fingerPins[i], i); // Read and process data from the sensor
    Serial.print(data); // Print the processed angle data

    if(i < NUM_SENSORS - 1) {
      Serial.print(','); // Print a comma if not the last sensor
    } else {
      Serial.print(';'); // Print a semicolon after the last sensor data
    }
  }

  // Print additional zero values for padding
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

  // Unity needs a delay to read data
  delay(30); // 30 frames per second rate
}

// Function to read the output voltage from the specified pin,
// calculate the resistance of the flex sensor, determine the angle bent,
// and return the calculated angle
float flexSensor(int PIN, int finger){
  // Read the analog value from the sensor pin
  int voltage_read = analogRead(PIN);

  // Get max and min calibration values for the specific finger
  float max = CalibrationVals[0][finger];
  float min = CalibrationVals[1][finger];

  // Clamp the voltage reading to within the calibrated min/max range
  if(voltage_read < min){
    voltage_read = min;
  }
  if(voltage_read > max){
    voltage_read = max;
  }

  // Calculate the angle based on the clamped sensor reading
  float angle = 100.0 - (100.0 * (voltage_read - min)) / (max - min);

  /*
  // Optional adjustment for the thumb sensor (finger == 0)
  // Uncomment the following lines to apply specific adjustments
  if (finger == 0){
    angle -= 50;
    if (angle < 0){
      angle = 0;
    } 
  }
  */

  return angle; // Return the calculated angle
}

// Function to calibrate flex sensors at startup
void CalibrateFlexSensors(){
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
    for (int i = 0; i < NUM_SENSORS; i++) {
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
    for (int j = 0; j < NUM_SENSORS; j++) {
      Serial.print(CalibrationVals[i][j]); // Print each calibration value
      Serial.print(", ");
    }
    Serial.println();
  }
}
