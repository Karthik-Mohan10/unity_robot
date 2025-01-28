#include <ezButton.h> // The library to use for SW pin
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <Kalman.h>

// Wi-Fi credentials
const char* ssid = "ESP8266_Access_Point"; 
const char* password = "12345678";
WiFiServer server(80);

int IR = 2; // Output of IR sensor connected to digital pin D4
String IRState;

// Rotary encoder pins
#define CLK_PIN 13  // The ESP8266 pin D7 connected to the rotary encoder's CLK pin
#define DT_PIN 12   // The ESP8266 pin D6 connected to the rotary encoder's DT pin
#define SW_PIN 14   // The ESP8266 pin D5 connected to the rotary encoder's SW pin
#define DIRECTION_CW -1   // clockwise direction
#define DIRECTION_CCW 1  // counter-clockwise direction
#define DEBOUNCE_TIME 5  // Debounce time in milliseconds

// Sensor & filtering variables
//connect the SCL pin to D1 and the SDA pin to D2 for MPU6050
Adafruit_MPU6050 mpu;
Kalman kalmanPitch, kalmanRoll;
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
float accelBiasX = 0, accelBiasY = 0, accelBiasZ = 0;
const int calibrationSamples = 200;
float roll, pitch, yaw;
float elapsedTime, currentTime, previousTime;

// Pin for Capacitive touch sensor
#define ain A0
int touchInput = 0; 

//..................................................................
// // mpu angle calculation
// float gyroX, gyroY, gyroZ;
// float accAngleX, accAngleY, accAngleZ, gyroAngleX, gyroAngleY, gyroAngleZ;
// float AccX, AccY, AccZ;

// int IR = 0; // digital pin D3 has a Infrared attached to it.

// ..................................................................

// Rotary encoder variables
volatile int counter = 0;
volatile int direction = 0;
volatile unsigned long lastInterruptTime = 0;  // Stores the last interrupt timestamp
ezButton button(SW_PIN);  // create ezButton object for pin 7
int prev_counter;

// Low-Pass Filter factor (0.0 - 1.0), lower values filter more noise.......................new
const float alpha = 0.2;  
float prev_accX = 0, prev_accY = 0, prev_accZ = 0;
float prev_gyroX, prev_gyroY, prev_gyroZ;

// Function to calibrate Gyro
void calibrateGyro() {
  Serial.println("Calibrating Gyroscope...");
  for (int i = 0; i < calibrationSamples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    gyroBiasX += g.gyro.x;
    gyroBiasY += g.gyro.y;
    gyroBiasZ += g.gyro.z;
    delay(5);
  }
  gyroBiasX /= calibrationSamples;
  gyroBiasY /= calibrationSamples;
  gyroBiasZ /= calibrationSamples;
  Serial.println("Gyro Calibration Complete.");
}

void calibrateAccelerometer() {
  float totalX = 0.0, totalY = 0.0, totalZ = 0.0;

  Serial.println("Calibrating accelerometer...");
  Serial.println("Ensure the MPU6050 is stationary and flat!");

  for (int i = 0; i < calibrationSamples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    totalX += a.acceleration.x;
    totalY += a.acceleration.y;
    totalZ += a.acceleration.z;

    delay(5); // Small delay between samples
  }

  // Compute offsets
  accelBiasX = totalX / calibrationSamples;
  accelBiasY = totalY / calibrationSamples;
  accelBiasZ = (totalZ / calibrationSamples) - 9.81; // Gravity adjustment for Z-axis

  Serial.println("Calibration complete!");
  // Serial.print("Offsets (X, Y, Z): ");
  // Serial.print(accelXOffset); Serial.print(", ");
  // Serial.print(accelYOffset); Serial.print(", ");
  // Serial.println(accelZOffset);
}

// Function for rotary encoder
void ICACHE_RAM_ATTR encoderIRS() {
  unsigned long currentInterruptTime = millis();
  
  // Debounce check: Ignore rapid changes within DEBOUNCE_TIME ms
  if (currentInterruptTime - lastInterruptTime < DEBOUNCE_TIME) {
      return;
  }
  lastInterruptTime = currentInterruptTime;  // Update last interrupt time
  static int lastState = 0;
  int currentState = (digitalRead(CLK_PIN) << 1) | digitalRead(DT_PIN);
  
  if (currentState != lastState) {
    if ((lastState == 0b00 && currentState == 0b01) || 
        (lastState == 0b01 && currentState == 0b11) || 
        (lastState == 0b11 && currentState == 0b10) || 
        (lastState == 0b10 && currentState == 0b00)) {
      
      counter--;
      direction = DIRECTION_CW;   // value = 1(count decrease)
    } else {
      counter++;
      direction = DIRECTION_CCW;  // value = -1(count increase)
    }
    lastState = currentState;
  }
  else{
    //Serial.println("NO_ROTATION");
    direction = 0;
  }
}

void setup() {

  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);

  // Start the Wi-Fi access point
  WiFi.softAP(ssid, password);
  Serial.print("\nAccess Point \"");
  Serial.print(ssid);
  Serial.println("\" started");

  // Display the access point's IP address
  Serial.print("IP address:\t");
  Serial.println(WiFi.softAPIP());

  // Start the server
  server.begin();
  Serial.println("Server started");

  //IR Sensor
  pinMode(IR, INPUT);

  //Rotary Encoder
  pinMode(CLK_PIN, INPUT);
  pinMode(DT_PIN, INPUT);
  pinMode(SW_PIN, INPUT_PULLUP);
  button.setDebounceTime(50); // set debounce time to 50 milliseconds

  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  
  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1) delay(10);
  }
  else{
    Serial.println("MPU6050 Found!");
  }

  // Set MPU parameters
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ); //5, 10, 21, 44, 94, 184

  calibrateGyro();  // Calibrate Gyro
  calibrateAccelerometer(); //Calibrate Accelerometer

  // Interrupt function call for encoder
  attachInterrupt(digitalPinToInterrupt(CLK_PIN), encoderIRS, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DT_PIN), encoderIRS, CHANGE); // To determine the direction

  //MPU6050 event intialization
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Initialize Kalman filter with the first accelerometer angle
  kalmanPitch.setAngle(atan2(a.acceleration.y, a.acceleration.z) * 180 / PI);
  kalmanRoll.setAngle(atan2(-a.acceleration.x, a.acceleration.z) * 180 / PI);

  currentTime = millis();

  
  Serial.println("Waiting for client connection...");

  // delay(50);
}

void loop() {
  WiFiClient client = server.available();
  if (!client) return;  // Wait for a client to connect


  while (client.connected()) {  // Keep sending data while the client is connected
    static unsigned long lastUpdate = 0;
    if (millis() - lastUpdate > 50) {  // 20Hz update rate
      lastUpdate = millis();

      // MPU angle.......
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      //Get time
      previousTime = currentTime;
      currentTime = millis();
      elapsedTime = (currentTime - previousTime) / 1000.0;

      // Apply Low-Pass Filter (LPF) to accelerometer
      float accX = alpha * (a.acceleration.x - accelBiasX) + (1 - alpha) * prev_accX;
      float accY = alpha * (a.acceleration.y - accelBiasY) + (1 - alpha) * prev_accY;
      float accZ = alpha * (a.acceleration.z - accelBiasZ) + (1 - alpha) * prev_accZ;
      prev_accX = accX;
      prev_accY = accY;
      prev_accZ = accZ;

      // Calculate angles from accelerometer (roll and pitch)
      float accAngleX = atan2(accY, sqrt(accX * accX + accZ * accZ)) * 180 / PI;
      float accAngleY = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180 / PI;

      // Apply Kalman filter
      roll = kalmanRoll.getAngle(accAngleX, g.gyro.x * 180 / PI * elapsedTime, elapsedTime);
      pitch = kalmanPitch.getAngle(accAngleY, g.gyro.y * 180 / PI * elapsedTime, elapsedTime);

      // Complementary filter for yaw (no accelerometer correction)
      yaw = 0.98 * (yaw + (g.gyro.z - gyroBiasZ) * elapsedTime * 180 / PI) + 0.02 * yaw;


      // Print the values on the serial monitor
      Serial.print("Roll: ");
      Serial.print(roll);
      Serial.print("/");
      Serial.print("Pitch: ");
      Serial.print(pitch);
      Serial.print("/");
      Serial.print("Yaw: ");
      Serial.println(yaw);
      Serial.println();

      /* Print out the values */
      Serial.print("Acceleration X: ");
      Serial.print(a.acceleration.x);
      Serial.print(", Y: ");
      Serial.print(a.acceleration.y);
      Serial.print(", Z: ");
      Serial.print(a.acceleration.z);
      // Serial.println(" m/s^2");


      // IR Sensor
      // int IRState = digitalRead(IR);
      Serial.print("Object Detection: ");
      Serial.println(digitalRead(IR));
      IRState = digitalRead(IR) == 0 ? "CLOSE" : "OPEN";
      // if (IRState == 0){
      //   client.println("IR:CLOSE");
      // }
      // else if (IRState == 1){
      //   client.println("IR:OPEN");
      // }

      // button.loop();
      // String buttonState = button.isPressed() ? "TRUE" : "FALSE";

      // Touch Sensor
      touchInput=analogRead(ain);
      // if (inputVal>50 && inputVal<300) {
      //   Serial.println("JUMP");
      // }
      String buttonState = touchInput>50 && touchInput<300 ? "TRUE" : "FALSE";


      // Format and send data in a single packet
      String data = "ROLL:" + String(roll, 2) + "/PITCH:" + String(pitch, 2) + 
                    "/YAW:" + String(yaw, 2) + "/IR:" + IRState + 
                    "/TEMP:" + String(temp.temperature, 2) + 
                    "/ENCODER_COUNT:" + String(counter) +
                    "/ENCODER_DIR:" + String(direction) +
                    "/BUTTON:" + buttonState;
                    // "/BUTTON:" + String(button.isPressed());

      client.println(data);

      direction = 0;

      // Reset encoder direction only when a new movement is detected
      static int lastCounter = 0;
      if (lastCounter != counter) {
          lastCounter = counter; // Update last known count
          direction = 0; // Reset direction after reading the change
      }
    }
    

    if (prev_counter != counter) {

      Serial.println("Rotary Encoder:");
        // if (direction == DIRECTION_CW){
        //   Serial.print("CLOCKWISE");
        //   client.println("ENCODER:CLOCKWISE");
        // }
        // else if (direction== DIRECTION_CCW) {
        //   Serial.print("ANTICLOCKWISE");
        //   client.println("ENCODER:ANTICLOCKWISE");
        // }
      Serial.print(" - direction: ");
      Serial.println(direction);
      Serial.print(" - count: ");
      Serial.println(counter);

      prev_counter = counter;
    }

    // button.loop();  // MUST call the loop() function first
    // if (button.isPressed()) {
    //   Serial.println("BUTTON_PRESSED");
    //   client.println("/BUTTON:TRUE");
    // }
    // else{
    //   client.println("/BUTTON:FALSE");
    // }

  }

  Serial.println("Client disconnected.");
  client.stop();  // Stop after the client disconnects
}
