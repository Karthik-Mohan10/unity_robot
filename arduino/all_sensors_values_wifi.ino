//connect the SCL pin to D1 and the SDA pin to D2
#include <ezButton.h>  // The library to use for SW pin
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Encoder.h>
#include <ESP8266WiFi.h> // Include the Wi-Fi library
#include <Kalman.h> //Kalman Filter library

const char* ssid = "ESP8266_Access_Point"; // The name of the Wi-Fi network to create
const char* password = "12345678";           // Password for the network
WiFiServer server(80); // Create a server on port 80


#define CLK_PIN 13  // The ESP8266 pin D7 connected to the rotary encoder's CLK pin
#define DT_PIN 12   // The ESP8266 pin D6 connected to the rotary encoder's DT pin
#define SW_PIN 14   // The ESP8266 pin D5 connected to the rotary encoder's SW pin
#define DIRECTION_CW 0   // clockwise direction
#define DIRECTION_CCW 1  // counter-clockwise direction

int IR = 0; // digital pin D3 has a Infrared attached to it.
int counter = 0;
int direction = -1; //DIRECTION_CW;
int CLK_state;
int prev_CLK_state;

ezButton button(SW_PIN);  // create ezButton object for pin 7

Adafruit_MPU6050 mpu;

// Create Kalman filter objects for pitch and roll
Kalman kalmanPitch;
Kalman kalmanRoll;

// mpu angle calculation
float gyroX, gyroY, gyroZ;
float accAngleX, accAngleY, accAngleZ, gyroAngleX, gyroAngleY, gyroAngleZ;
float elapsedTime, currentTime, previousTime;
float roll, pitch, yaw;
float AccX, AccY, AccZ;
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
const int calibrationSamples = 200;

void calibrateGyro() {
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
}

void calibrateAccelerometer() {

}

void setup() {
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);

  Serial.println("\n");

  // Start the Wi-Fi access point
  WiFi.softAP(ssid, password);
  Serial.print("Access Point \"");
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
  
  //Positional Encoder
  button.setDebounceTime(50);  // set debounce time to 50 milliseconds
  pinMode(CLK_PIN, INPUT);
  pinMode(DT_PIN, INPUT);
  // read the initial state of the rotary encoder's CLK pin
  prev_CLK_state = digitalRead(CLK_PIN);

  //MPU6050
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  else{
    Serial.println("MPU6050 Found!");
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ); //5, 10, 21, 44, 94, 184

  Serial.println("");

  // Initialize Kalman filter
  kalmanPitch.setAngle(0); // Initialize pitch angle
  kalmanRoll.setAngle(0);  // Initialize roll angle

  calibrateGyro();

  currentTime = millis();

  delay(50);
}

void loop() {
  
  // Check if a client has connected
  WiFiClient client = server.available();

  while(client.connected()) {

    static unsigned long lastUpdate = 0;
    if (millis() - lastUpdate>500){
      // IR Sensor
      lastUpdate = millis();
      int IRState = digitalRead(IR);
      Serial.print("Object Detection: ");
      Serial.println(IRState);
      if (IRState == 0){
        client.println("IR:CLOSE");
      }
      else if (IRState == 1){
        client.println("IR:OPEN");
      }

      //MPU6050
      //sensors_event_t a, g, temp;
      //mpu.getEvent(&a, &g, &temp);

      ///////////////////////// MPU angle ////////////

      //Get time
      previousTime = currentTime;        // Previous time is stored before the actual time read
      currentTime = millis();            // Current time actual time read
      elapsedTime = (currentTime - previousTime) / 1000.0; // Divide by 1000.0 to get seconds

      // Serial.print("Elapsed Time: ");
      // Serial.println(elapsedTime);

      //MPU6050
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);

      //Uncomment to read values to fix zero error
      // Serial.println("Accelerometer");
      // Serial.println(a.acceleration.x);
      // Serial.println(a.acceleration.y);
      // Serial.println(a.acceleration.z);
      // Serial.println();
      // Serial.println("Gyroscope");
      // Serial.println(g.gyro.x);
      // Serial.println(g.gyro.y);
      // Serial.println(g.gyro.z);
      // Serial.println();

      //Accelerometer Angles
      AccX = a.acceleration.x; // X-axis value - zero error
      AccY = a.acceleration.y; // Y-axis value
      AccZ = a.acceleration.z; // Z-axis value

      // Calculating Roll and Pitch from the accelerometer data
      accAngleX = atan2(AccY, sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI;
      accAngleY = atan2((-1 * AccX), sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI;


      //Gyro Angles
      gyroX = (g.gyro.x - gyroBiasX) * 180 / PI;
      gyroY = (g.gyro.y - gyroBiasY) * 180 / PI;
      gyroZ = (g.gyro.z - gyroBiasZ) * 180 / PI;
      gyroAngleX = gyroAngleX + gyroX * elapsedTime; // deg/s * s = deg
      gyroAngleY = gyroAngleY + gyroY * elapsedTime;


      // Apply Kalman filter for pitch and roll
      roll = kalmanRoll.getAngle(accAngleX, gyroAngleX, elapsedTime);
      pitch = kalmanPitch.getAngle(accAngleY, gyroAngleY, elapsedTime);
      
      //Accelerometer reading cannot be used for the values along the line of acc. due to gravity
      yaw =  yaw + gyroZ * elapsedTime;

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
      Serial.println(" m/s^2");

      // Serial.print("Rotation X: ");
      // Serial.print(g.gyro.x);
      // Serial.print(", Y: ");
      // Serial.print(g.gyro.y);
      // Serial.print(", Z: ");
      // Serial.print(g.gyro.z);
      // Serial.println(" rad/s");

      Serial.print("Temperature: ");
      Serial.print(temp.temperature);
      Serial.println(" degC");

      Serial.println("");
      client.print("AccelX:");
      client.println(a.acceleration.x*10);
      client.print("AccelY:");
      client.println(a.acceleration.y*10);
      client.print("AccelZ:");
      client.println(a.acceleration.z*10);
      client.print("GyroX:");
      client.println(roll);
      client.print("GyroY:");
      client.println(pitch);
      client.print("GyroZ:");
      client.println(yaw);
      client.print("TEMP:");
      client.println(String(temp.temperature,2));
      // if (direction == -1){
      //   client.println("ENCODER:CLOCKWISE");
      // }
    }
      
    button.loop();  // MUST call the loop() function first

    static int lastState = 0; // Previous state of the encoder
    static int currentState = 0; // Current state of the encoder

    currentState = (digitalRead(CLK_PIN) << 1) | digitalRead(DT_PIN); // Read CLK and DT
    
    if (currentState != lastState) { // State has changed
      if ((lastState == 0b00 && currentState == 0b01) || 
          (lastState == 0b01 && currentState == 0b11) || 
          (lastState == 0b11 && currentState == 0b10) || 
          (lastState == 0b10 && currentState == 0b00)) {
        // Clockwise
        counter--;
        direction = DIRECTION_CCW;
      } else if ((lastState == 0b00 && currentState == 0b10) || 
                (lastState == 0b10 && currentState == 0b11) || 
                (lastState == 0b11 && currentState == 0b01) || 
                (lastState == 0b01 && currentState == 0b00)) {
        // Counter-clockwise
        counter++;
        direction = DIRECTION_CW;
      }
      Serial.print("Rotary Encoder:: direction: ");
      if (direction == DIRECTION_CW){
        Serial.print("CLOCKWISE");
        client.println("ENCODER:CLOCKWISE");
      }
      else {
        Serial.print("ANTICLOCKWISE");
        client.println("ENCODER:ANTICLOCKWISE");
      }
      Serial.print(" - count: ");
      Serial.println(counter);
      
    }

    else{
      //Serial.println("NO_ROTATION");
      direction = -1;
    }


    lastState = currentState; // Save the current state for next iteration
    if (button.isPressed()) {
      Serial.println("The button is pressed");
      client.println("ENCODER:BUTTON_PRESSED");
    }
    
    //delay(10);
  }
  client.stop();  
}
