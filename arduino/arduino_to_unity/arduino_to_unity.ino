#include <ezButton.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <Kalman.h>
#include <ArduinoJson.h>

// Wi-Fi credentials
const char* ssid = "ESP8266_Access_Point"; 
const char* password = "12345678";
WiFiServer server(80);

// Sensor Connections and Variables

// Infrared Sensor
#define IR 2 // Output of IR sensor connected to digital pin D4
String IRState;

// Rotary Encoder
#define CLK_PIN 13  // The ESP8266 pin D7 connected to the rotary encoder's CLK pin
#define DT_PIN 12   // The ESP8266 pin D6 connected to the rotary encoder's DT pin
#define SW_PIN 14   // The ESP8266 pin D5 connected to the rotary encoder's SW pin
#define DIRECTION_CW -1   // Clockwise direction
#define DIRECTION_CCW 1  // Counter-clockwise direction
#define DEBOUNCE_TIME 5  // Debounce time in milliseconds
volatile int counter = 0;
volatile int direction = 0;
volatile unsigned long lastInterruptTime = 0;  // Stores the last interrupt timestamp
ezButton button(SW_PIN);  // Create ezButton object for pin 7
int prev_counter;

// Capacitive Sensor
#define ain A0  // Capacitive sensor is connected to Analog pin A0 and Ground
int touchInput = 0; 

// Gyroscope and Accelerometer
// Connect the SCL pin to D1 and the SDA pin to D2 for MPU6050
Adafruit_MPU6050 mpu;
Kalman kalmanPitch, kalmanRoll;
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
float accelBiasX = 0, accelBiasY = 0, accelBiasZ = 0;
const int calibrationSamples = 200;
float roll, pitch, yaw;
float elapsedTime, currentTime, previousTime;

// Low-Pass Filter factor (0.0 - 1.0). Lower values of alpha filter more noise
const float alpha = 0.2;  
float prev_accX = 0, prev_accY = 0, prev_accZ = 0;
float prev_gyroX, prev_gyroY, prev_gyroZ;

// Function to calibrate gyroscope
void calibrateGyro() {
  Serial.println("Ensure the MPU6050 is stationary and flat!");
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
  Serial.println("Gyroscope Calibration Complete!");
}

// Function to calibrate accelerometer
void calibrateAccelerometer() {
  float totalX = 0.0, totalY = 0.0, totalZ = 0.0;

  Serial.println("Calibrating Accelerometer...");

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

  Serial.println("Accelerometer Calibration Complete!");
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
    } 
    else {
      counter++;
      direction = DIRECTION_CCW;  // value = -1(count increase)
    }
    lastState = currentState;
  }
  else {
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

  // IR Sensor
  pinMode(IR, INPUT);

  // Rotary Encoder
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
  else {
    Serial.println("MPU6050 Found!");
  }

  // Set MPU parameters
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  // Call calibration functions
  calibrateGyro();
  calibrateAccelerometer();

  // Interrupt function call for encoder
  attachInterrupt(digitalPinToInterrupt(CLK_PIN), encoderIRS, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DT_PIN), encoderIRS, CHANGE); // To determine the direction

  // MPU6050 event intialization
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Initialize Kalman filter with the first accelerometer angle
  kalmanPitch.setAngle(atan2((a.acceleration.y - accelBiasY), (a.acceleration.z - accelBiasZ)) * 180 / PI);
  kalmanRoll.setAngle(atan2((-a.acceleration.x - accelBiasX), (a.acceleration.z - accelBiasZ)) * 180 / PI);

  Serial.println("Waiting for client connection...");

  delay(50);
}

void loop() {
  WiFiClient client = server.available();
  if (!client) return;  // Wait for a client to connect

  while (client.connected()) {  // Keep sending data while the client is connected
    static unsigned long lastUpdate = 0;
    if (millis() - lastUpdate > 50) {  // 20Hz update rate
      lastUpdate = millis();

      // Calculate orientation angles from MPU6050 readings
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);

      // Get time interval for gyro angle calculation
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
      roll = kalmanRoll.getAngle(accAngleX, (g.gyro.x - gyroBiasX) * 180 / PI * elapsedTime, elapsedTime);
      pitch = kalmanPitch.getAngle(accAngleY, (g.gyro.y - gyroBiasY) * 180 / PI * elapsedTime, elapsedTime);

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

      // IR Sensor
      IRState = digitalRead(IR) == 0 ? "CLOSE" : "OPEN";

      // Touch Sensor
      // The sensor registers a touch when the Analog pin outputs a value between 100 and 300
      touchInput=analogRead(ain);
      String buttonState = touchInput>100 && touchInput<300 ? "TRUE" : "FALSE";

      // Store all values to be sent as a JSON
      StaticJsonDocument<256> jsonDoc;
      jsonDoc["ROLL"] = String(roll, 2);
      jsonDoc["PITCH"] = String(pitch, 2);
      jsonDoc["YAW"] = String(yaw, 2);
      jsonDoc["IR"] = IRState;
      jsonDoc["TEMP"] = String(temp.temperature, 2);
      jsonDoc["ENCODER_COUNT"] = String(counter);
      jsonDoc["ENCODER_DIR"] = String(direction);
      jsonDoc["BUTTON"] = buttonState;

      String jsonRes;

      serializeJson(jsonDoc, jsonRes);

      // Send values to Unity
      client.println(jsonRes);

      // Reset encoder direction only when a new movement is detected
      static int lastCounter = 0;
      if (lastCounter != counter) {
          lastCounter = counter; // Update last known count
          direction = 0; // Reset direction after reading the change
      }
    }
    
    if (prev_counter != counter) {
      prev_counter = counter;
    }
  }

  Serial.println("Client disconnected.");
  client.stop();  // Stop after the client disconnects
}
