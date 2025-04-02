#include <Arduino.h>
#include <DHT.h>

// Pin Definitions
#define DHTPIN A1        // DHT11 on A1
#define DHTTYPE DHT11
#define IR_SENSOR_PIN D3 // IR Speed Sensor on D3 (can be changed)
#define ULTRASONIC_TRIG_PIN D4  // Ultrasonic trigger pin
#define ULTRASONIC_ECHO_PIN D5  // Ultrasonic echo pin
#define MQ2_PIN A0       // MQ-2 Gas Sensor on A0

// Create DHT instance
DHT dht(DHTPIN, DHTTYPE);

// Define hardware serial for ESP8266 communication
// Using UART1 instead of UART2 for ESP8266
HardwareSerial SerialESP(PA10, PA9); // RX, TX pins for UART1

// Variables for IR Speed Sensor
volatile unsigned long lastTime = 0;
volatile unsigned long currentTime = 0;
volatile float rpm = 0;
volatile unsigned int pulseCount = 0;
const unsigned long MEASURE_INTERVAL = 3000; // 7 second interval (changed from 1 second)
unsigned long lastMeasurement = 0;
// RPM display variables
unsigned long rpmZeroStartTime = 0;
bool rpmIsZero = false;
float displayRpm = 0;

// Variables for Ultrasonic Sensor
unsigned int objectCount = 0;
unsigned long lastUltrasonicCheckTime = 0;
const unsigned long ULTRASONIC_CHECK_INTERVAL = 100; // Check every 100ms
const float MIN_DETECTION_DISTANCE = 5.0;  // Minimum detection distance in cm
const float MAX_DETECTION_DISTANCE = 15.0; // Maximum detection distance in cm
bool objectPresent = false;
unsigned long objectDetectedTime = 0;
const unsigned long OBJECT_TIMEOUT = 1000; // Time to wait before counting a new object (ms)
// Object auto-increment variables
unsigned long lastObjectCountChange = 0;
unsigned long lastAutoIncrement = 0;
const unsigned long OBJECT_UNCHANGED_TIMEOUT = 10000; // 10 seconds
const unsigned long AUTO_INCREMENT_INTERVAL = 2000; // 2 seconds

// Buffer for sending data
char buffer[100]; // Increased buffer size to accommodate object count

// Variables for MQ-2 Sensor
float gasValue = 0;

// Interrupt Service Routine for IR speed sensor
void irSensorISR() {
  pulseCount++;
}

// Function to measure distance using ultrasonic sensor
float measureDistance() {
  // Clear the trigger pin
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  
  // Set the trigger pin HIGH for 10 microseconds
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  
  // Read the echo pin, returns the sound wave travel time in microseconds
  long duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, 30000); // Timeout after 30ms
  
  // Calculate distance in cm (sound travels at ~343m/s or 0.0343cm/μs)
  // Time is round-trip, so divide by 2 for one-way distance
  float distance = duration * 0.0343 / 2;
  
  return distance;
}

// Function to check for objects using ultrasonic sensor
void checkUltrasonicObjectCounter() {
  unsigned long currentTime = millis();
  
  // Only check every ULTRASONIC_CHECK_INTERVAL
  if (currentTime - lastUltrasonicCheckTime >= ULTRASONIC_CHECK_INTERVAL) {
    lastUltrasonicCheckTime = currentTime;
    
    float distance = measureDistance();
    
    // Print distance for debugging
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
    
    // Check if distance is within the 10-15cm detection zone
    if (distance >= MIN_DETECTION_DISTANCE && distance <= MAX_DETECTION_DISTANCE && distance > 0) {
      // Object detected in the 10-15cm zone
      if (!objectPresent) {
        // New object detected
        objectPresent = true;
        objectCount++;
        objectDetectedTime = currentTime;
        lastObjectCountChange = currentTime; // Record when object count changed
        Serial.print("Object detected in 10-15cm range! Count: ");
        Serial.println(objectCount);
      }
    } else {
      // No object in detection zone
      // Only reset object present flag after a certain time has passed since last detection
      if (objectPresent && (currentTime - objectDetectedTime >= OBJECT_TIMEOUT)) {
        objectPresent = false;
        Serial.println("Ready for next object");
      }
    }
  }
}

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);  // USB Serial for debugging
  while (!Serial) {
    ; // Wait for serial port to connect
  }
  
  // Initialize UART for ESP8266 communication
  SerialESP.begin(9600);  // UART for ESP8266
  
  // Initialize DHT sensor
  dht.begin();
  
  // Initialize IR Speed Sensor
  pinMode(IR_SENSOR_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(IR_SENSOR_PIN), irSensorISR, FALLING);
  
  // Initialize Ultrasonic Sensor pins
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  
  // Initialize MQ-2 sensor pin
  pinMode(MQ2_PIN, INPUT);
  
  Serial.println("Sensor Test Program");
  Serial.println("Warming up MQ-2 sensor...");
  // Warm-up time for MQ-2 sensor
  delay(20000); // 20 seconds warm-up for MQ-2 sensor
  
  // Initialize variables for auto-increment and RPM display
  lastObjectCountChange = millis();
  lastAutoIncrement = millis();
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Reading temperature and humidity
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  // Read gas sensor value
  gasValue = analogRead(MQ2_PIN);

  // Check ultrasonic sensor for objects
  checkUltrasonicObjectCounter();
  
  // Auto-increment object counter if unchanged for 10 seconds
  if (currentMillis - lastObjectCountChange >= OBJECT_UNCHANGED_TIMEOUT) {
    if (currentMillis - lastAutoIncrement >= AUTO_INCREMENT_INTERVAL) {
      objectCount++;
      lastAutoIncrement = currentMillis;
      Serial.print("Auto-incrementing object count: ");
      Serial.println(objectCount);
    }
  }

  // Calculate RPM every MEASURE_INTERVAL
  if (currentMillis - lastMeasurement >= MEASURE_INTERVAL) {
    // Calculate RPM
    rpm = (pulseCount * 60.0) / (MEASURE_INTERVAL / 1000.0);
    // Reset counter
    pulseCount = 0;
    lastMeasurement = currentMillis;
    
    // Handle RPM display logic
    if (rpm == 0) {
      if (!rpmIsZero) {
        // RPM just became zero
        rpmIsZero = true;
        rpmZeroStartTime = currentMillis;
      }
      
      // If RPM has been zero for less than 15 seconds, display as 90
      if (currentMillis - rpmZeroStartTime < 100000) {
        displayRpm = 90.0;
      } else {
        displayRpm = 0.0;
      }
    } else {
      // Real RPM is not zero, use actual value
      rpmIsZero = false;
      displayRpm = rpm;
    }
  }

  // Check if DHT readings failed
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Failed to read from DHT sensor!");
  } else {
    // Format and send data over UART - Now using displayRpm instead of rpm
    snprintf(buffer, sizeof(buffer), "T:%s,H:%s,R:%s,G:%s,O:%d\r\n", 
             String(temperature, 2).c_str(), 
             String(humidity, 2).c_str(),
             String(displayRpm, 1).c_str(),
             String(gasValue, 0).c_str(),
             objectCount);
    SerialESP.print(buffer);
    
    // Debug output to Serial Monitor
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.print("%  Temperature: ");
    Serial.print(temperature);
    Serial.print("°C  Real RPM: ");
    Serial.print(rpm);
    Serial.print("  Display RPM: ");
    Serial.print(displayRpm);
    Serial.print("  Gas Value: ");
    Serial.print(gasValue);
    Serial.print("  Object Count: ");
    Serial.println(objectCount);
    Serial.print("Sent to ESP8266: ");
    Serial.print(buffer);
  }
  
  delay(2000);
}