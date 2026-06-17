#include <Wire.h>

//constants
const int LED_PIN = 21;                 //GPIO pin connected to the LED
const int MPU_ADDR = 0x68;              //default I2C address of the MPU6050
const float ACCEL_THRESHOLD = 15.0;     //impact threshold in m/s^2

const String RESTART_PHRASE = "clear";  //resumes monitoring after an alert
const String START_PHRASE = "start";    //turns the system on
const String STOP_PHRASE = "stop";      //turns the system off

//state
bool systemRunning = false;       //true = device is on
bool thresholdActive = false;      //true = actively monitoring for impacts

//peak tracking (stores the highest impact reading within the current 50ms window)
float PeakX = 0.0, PeakY = 0.0, PeakZ = 0.0, PeakSum = 0.0;

//window timing
unsigned long lastWindowResetTime = 0;          
const unsigned long WINDOW_DURATION_MS = 50;   //report peaks every 50ms

//rolling gravity baseline (tracks the gravity vector so it can be subtracted from live readings)
float baselineX = 0, baselineY = 0, baselineZ = 9.8;
const float ALPHA = 0.02; //how fast the baseline adapts (2% per reading (1000 readings per second))

//resets all peak values to zero
void resetPeaks() {
  PeakX = PeakY = PeakZ = PeakSum = 0;
}

//executes serial commands from the user
void handleCommand(String input) {
  //normalizes inputs: removes whitespace, makes lowercase
  input.trim();
  input.toLowerCase();
  input.replace(" ", "");

  if (input == START_PHRASE) {
    if (systemRunning) {
      //ignore if already on
      Serial.println("Already on. Use 'clear' to resume monitoring or 'stop' to turn off.");
      return;
    }
    systemRunning = true;
    thresholdActive = true;
    resetPeaks();
    lastWindowResetTime = millis(); //starts the first window
    digitalWrite(LED_PIN, LOW);
    Serial.println("System on. Monitoring started.");
  }
  else if (input == STOP_PHRASE) {
    if (!systemRunning) {
      //ignore if already off
      Serial.println("System is already off.");
      return;
    }
    systemRunning = false;
    thresholdActive = false;
    digitalWrite(LED_PIN, LOW);
    Serial.println("System off.");
  }
  else if (input == RESTART_PHRASE) {
    if (!systemRunning) {
      //can't resume if the device isn't on
      Serial.println("System is off. Use 'start' first.");
      return;
    }
    if (thresholdActive) {
      //already monitoring, nothing to resume
      Serial.println("Already monitoring.");
      return;
    }
    //system is on but paused after an alert, so resume monitoring
    thresholdActive = true;
    resetPeaks();
    lastWindowResetTime = millis();
    digitalWrite(LED_PIN, LOW);
    Serial.println("Monitoring resumed.");
  }
  else {
    //unknown commands
    Serial.print("Unknown command: '");
    Serial.print(input);
    Serial.println("'. Use start, stop, or clear.");
  }
}

//setup
void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);
  while (!Serial) delay(10); //wait for serial monitor to open

  Serial.println("Initializing MPU6050...");
  Wire.begin(); //start I2C bus
  delay(100);   //give sensor time to power up

  //wake up the MPU6050 (register 0x6B = power management, 0 = wake up)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0); 
  byte error = Wire.endTransmission();

  if (error == 0) {
    Serial.println("MPU6050 connected. Type 'start' to begin.");
  } 
  else {
    Serial.print("Connection failed, error code: ");
    Serial.println(error);
    while (1) delay(10); //halt if sensor not found
  }

  //configure DLPF (Digital Low-Pass Filter) (register 0x1A, value 3 = ~44Hz bandwidth)
  //smooths out high frequency electrical noise before data reaches the code  
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);
  Wire.write(3);
  Wire.endTransmission();

  //set accelerometer range to +-16g (register 0x1c, value 0x18)
  //scale factor becomes 2048 LSB/g instead of the default 16384 at +-2g
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x18);
  Wire.endTransmission();

  digitalWrite(LED_PIN, LOW);
}

//loop (actual process)
void loop() {
  //check for incoming serial commands each cycle
  if (Serial.available() > 0) {
    handleCommand(Serial.readStringUntil('\n'));
  }

  //skip IMU reading if system is off or paused after alert
  if (!systemRunning || !thresholdActive) return;

  //request 6 bytes of accelerometer data starting at register 0x3B
  //false = keep I2C bus active between write and read
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  //guard: skip this cycle if I2C glitched and returned fewer than 6 bytes
  if (Wire.available() < 6) return;

  //each axis is two bytes (high byte + low byte); shift and OR to assemble
  int16_t rawX = (Wire.read() << 8) | Wire.read();
  int16_t rawY = (Wire.read() << 8) | Wire.read();
  int16_t rawZ = (Wire.read() << 8) | Wire.read();

  //convert raw integers to m/s^2: divide by scale factor (2048 at +-16g), multiply by 9.8
  float x = (rawX / 2048.0) * 9.8;
  float y = (rawY / 2048.0) * 9.8;
  float z = (rawZ / 2048.0) * 9.8;

  //compute magnitude of the current gravity baseline vector
  float baselineMagnitude = sqrt(baselineX * baselineX + baselineY * baselineY + baselineZ * baselineZ);
  
  //compute total acceleration magnitude of live reading
  float totalAccel = sqrt(x*x + y*y + z*z);

  //only update baseline during quiet moments (no impact happening)
  //prevents an impact spike from corrupting the gravity reference
  if (totalAccel - baselineMagnitude < 2.0) {
    baselineX = ALPHA * x + (1 - ALPHA) * baselineX;
    baselineY = ALPHA * y + (1 - ALPHA) * baselineY;
    baselineZ = ALPHA * z + (1 - ALPHA) * baselineZ; 
  }

  //subtract gravity baseline component by component to isolate impact
  float impactX = x - baselineX;
  float impactY = y - baselineY;
  float impactZ = z - baselineZ;
  float impactAccel = sqrt(impactX * impactX + impactY * impactY + impactZ * impactZ);

  //update peaks if this reading is the strongest so far in this window
  if (impactAccel > PeakSum) {
    PeakSum = impactAccel;
    PeakX = impactX;
    PeakY = impactY;
    PeakZ = impactZ;
  }

  //at the end of each 50ms window, report and check threshold
  if (millis() - lastWindowResetTime >= WINDOW_DURATION_MS) {
    Serial.print("X: "); Serial.print(PeakX);
    Serial.print(" | Y: "); Serial.print(PeakY);
    Serial.print(" | Z: "); Serial.print(PeakZ);
    Serial.print(" | Sum: "); Serial.println(PeakSum);
  
    if (PeakSum > ACCEL_THRESHOLD) {
      //pause monitoring and trigger alert; system stays on for BLE later
      thresholdActive = false;
      digitalWrite(LED_PIN, HIGH);
      Serial.println("=========================================");
      Serial.print(PeakSum);
      Serial.println(" m/s^2 --> THRESHOLD EXCEEDED!");
      Serial.println("=========================================");
    }

    //reset peaks and restart the window timer
    resetPeaks();
    lastWindowResetTime = millis();
  }
}