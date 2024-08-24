#include "I2Cdev.h"
#include <math.h>
#include "MPU6050_6Axis_MotionApps20.h"

// Use the Wire library for I2C communication if specified
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu; // Create an instance of the MPU6050 class

#define OUTPUT_READABLE_WORLDACCEL  // Define this to output world acceleration data

// MPU control/status variables
bool dmpReady = false;  // True if the DMP is initialized successfully
uint8_t mpuIntStatus;   // Interrupt status from MPU
uint8_t devStatus;      // Return status of device initialization
uint16_t packetSize;    // Expected size of DMP data packets
uint16_t fifoCount;     // Number of bytes currently in the FIFO buffer
uint8_t fifoBuffer[64]; // Buffer to store FIFO data

// Variables for orientation and motion data
Quaternion q;           // Quaternion to store orientation data
VectorInt16 aa;         // Accelerometer sensor measurements
VectorInt16 aaReal;     // Gravity-free accelerometer measurements
VectorInt16 aaWorld;    // World-frame accelerometer measurements
VectorFloat gravity;    // Gravity vector
float euler[3];         // Euler angles (psi, theta, phi)
float ypr[3];           // Yaw, pitch, roll angles
float acc[3];           // Acceleration vector
float lastAcc[3] = {0.0f, 0.0f, 0.0f}; // Last acceleration values
float vel[3] = {0.0f, 0.0f, 0.0f};     // Velocity vector
unsigned long lastTime = 0;  // Last time the loop was executed
float dt = 0;                // Delta time for integration
float connected;             // Variable to check if the sensor is connected

void setup() {
  // Initialize I2C communication
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // Set I2C clock speed to 400kHz
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true); // Setup Fastwire if used
  #endif

  Serial.begin(115200);  // Start serial communication at 115200 baud
  while (!Serial);       // Wait for the serial port to connect

  // Initialize the MPU6050 device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // Verify the connection to the MPU6050
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // Load and configure the DMP (Digital Motion Processor)
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // Set gyro and accelerometer offsets
  mpu.setXGyroOffset(-197);
  mpu.setYGyroOffset(-24);
  mpu.setZGyroOffset(-23);
  mpu.setXAccelOffset(-388);
  mpu.setYAccelOffset(-535);
  mpu.setZAccelOffset(1507);

  // Check if DMP initialization was successful
  if (devStatus == 0) {
    // Calibrate the accelerometer and gyro
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets(); // Print the offsets

    // Enable the DMP
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // Indicate that the DMP is ready
    Serial.println(F("DMP ready! Waiting for first data..."));
    dmpReady = true;

    // Get the expected packet size for later use
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // If DMP initialization failed, print the error code
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // Record the current time
  lastTime = millis();
}

void loop() {
  // If DMP is not ready, exit the loop
  if (!dmpReady) return;

  // Check if there's a new packet from the FIFO buffer
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    #ifdef OUTPUT_READABLE_WORLDACCEL
      // Read the quaternion data from the DMP
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      
      // Print placeholder zero values (for data structure consistency)
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

      // Print quaternion data (x, y, z components)
      Serial.print(q.x);
      Serial.print(",");
      Serial.print(q.y);
      Serial.print(",");
      Serial.print(q.z);
      Serial.print(';');
        
      // Print velocity data (x, y, z components)
      Serial.print(vel[0]);
      Serial.print(',');
      Serial.print(vel[1]);
      Serial.print(',');
      Serial.print(vel[2]);
      Serial.println("");
    #endif

    // Delay to simulate a 30 FPS frame rate (for Unity or similar visualization tools)
    delay(30);
  }
}
