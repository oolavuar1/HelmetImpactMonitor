#include <Wire.h>

//constants
const int LED_PIN = 21;                 //GPIO pin connected to the LED
const int MPU_ADDR = 0x68;              //default I2C address of the MPU6050
const float ACCEL_THRESHOLD = 15.0;     //impact threshold in m/s^2
const float GYRO_THRESHOLD = 1500;      //rotational threshold in dps

const String RESTART_PHRASE = "clear";  //resumes monitoring after an alert
const String START_PHRASE = "start";    //turns the system on
const String STOP_PHRASE = "stop";      //turns the system off
const String CAL_PHRASE = "calibrate";  //triggers an imu calibration

//state
bool systemRunning = false;       //true = device is on
bool thresholdActive = false;      //true = actively monitoring for impacts

//peak tracking (stores the highest impact reading within the current 50ms window)
float PeakX = 0.0, PeakY = 0.0, PeakZ = 0.0, PeakSum = 0.0; //accel peaks
float PeakGX = 0.0, PeakGY = 0.0, PeakGZ = 0.0, PeakGyro = 0.0; //gyro peaks

//window timing
unsigned long lastWindowResetTime = 0;          
const unsigned long WINDOW_DURATION_MS = 50;   //report peaks every 50ms

//rolling gravity baseline (tracks the gravity vector so it can be subtracted from live readings)
float baselineX = 0, baselineY = 0, baselineZ = 0;
const float ALPHA = 0.02; //how fast the baseline adapts (2% per reading (1000 readings per second))

//gyro drift offsets (calculated at startup, subtracted every loop)
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;

//resets all peak values to zero
void resetPeaks() {
  PeakX = PeakY = PeakZ = PeakSum = 0;
  PeakGX = PeakGY = PeakGZ = PeakGyro = 0;
}

//triggers an alert, pauses monitoring, and lights the LED
//takes a message string so accel and gyro can report differently
void triggerAlert(String reason, float value, String unit) {
  thresholdActive = false;
  digitalWrite(LED_PIN, HIGH);
  Serial.println("=========================================");
  Serial.print(value);
  Serial.print(" ");
  Serial.print(unit);
  Serial.print(" --> ");
  Serial.print(reason);
  Serial.println(" THRESHOLD EXCEEDED!");
  Serial.println("=========================================");
}

//samples both sensors for 2 seconds at rest to zero out offsets
//sets the gravity baseline and stores gyro drift offsets
//hold the device completely still during this window
void calibrateIMU() {
  Serial.println("Calibrating... keep sensor still.");

  long calGX = 0, calGY = 0, calGZ = 0;
  long calX = 0, calY = 0, calZ = 0;
  int samples = 0;
  unsigned long calStart = millis();

  while (millis() - calStart < 2000) {
    //read accel
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);
    if (Wire.available() >= 6) {
      calX += (int16_t)((Wire.read() << 8) | Wire.read());
      calY += (int16_t)((Wire.read() << 8) | Wire.read());
      calZ += (int16_t)((Wire.read() << 8) | Wire.read());
    }

    //read gyro
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);
    if (Wire.available() >= 6) {
      calGX += (int16_t)((Wire.read() << 8) | Wire.read());
      calGY += (int16_t)((Wire.read() << 8) | Wire.read());
      calGZ += (int16_t)((Wire.read() << 8) | Wire.read());
    }

    samples++;
    delay(2);
  }

  //seed the accel baseline directly from calibration averages
  //replaces a guess of (0, 0, 9.8) with actual resting values
  baselineX = ((float)calX / samples / 2048.0) * 9.8;
  baselineY = ((float)calY / samples / 2048.0) * 9.8;
  baselineZ = ((float)calZ / samples / 2048.0) * 9.8;

  //store gyro averages as offsets to subtract every loop
  gyroOffsetX = (float)calGX / samples / 16.4;
  gyroOffsetY = (float)calGY / samples / 16.4;
  gyroOffsetZ = (float)calGZ / samples / 16.4;

  resetPeaks();
  lastWindowResetTime = millis();

  Serial.println("Calibration completed. Type 'start' to begin, or recalibrate using 'calibrate'.");
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
  else if (input == CAL_PHRASE) {
    if (systemRunning) {
      //system must be off to calibrate, otherwise IMU will continue to send readings
      Serial.println("Stop the system before calibrating. Use 'stop' first.");
      return;
    }
    calibrateIMU();
  }
  else {
    //unknown commands
    Serial.print("Unknown command: '");
    Serial.print(input);
    Serial.println("'. Use start, stop, clear, or calibrate");
  }
}

//setup
void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);
  delay(2000);
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
    Serial.println("MPU6050 connected. Please wait for two seconds while it calibrates...");

    //configure DLPF (Digital Low-Pass Filter) (register 0x1A, value 3 = ~44Hz bandwidth)
    //smooths out high frequency noise before data reaches the code  
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

    //set gyro range to +-2000 dps (register 0x1B, value 0x18)
    //scale factor set to 16.4 LSP/(dps)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1B);
    Wire.write(0x18);
    Wire.endTransmission();

    calibrateIMU();
  } 
  else {
    Serial.print("Connection failed, error code: ");
    Serial.println(error);
    while (1) delay(10); //halt if sensor not found
  }

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

  //read accelerometer data starting at register 0x3B
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  //guard
  if (Wire.available() < 6) return;

  int16_t rawX = (Wire.read() << 8) | Wire.read();
  int16_t rawY = (Wire.read() << 8) | Wire.read();
  int16_t rawZ = (Wire.read() << 8) | Wire.read();

  //convert to m/s^2: divide by scale factor (2048 at +-16g), multiply by 9.8
  float x = (rawX / 2048.0) * 9.8;
  float y = (rawY / 2048.0) * 9.8;
  float z = (rawZ / 2048.0) * 9.8;

  //read gyroscope data starting at register 0x43
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  //guard
  if (Wire.available() < 6) return;

  int16_t rawGX = (Wire.read() << 8) | Wire.read();
  int16_t rawGY = (Wire.read() << 8) | Wire.read();
  int16_t rawGZ = (Wire.read() << 8) | Wire.read();

  //convert to dps: divide by scale factor (16.4 at +-2000 dps) and subtract calibration offsets to remove drift
  float gx = (rawGX / 16.4) - gyroOffsetX;
  float gy = (rawGY / 16.4) - gyroOffsetY;
  float gz = (rawGZ / 16.4) - gyroOffsetZ;

  //compute magnitude of the current gravity baseline vector
  float baselineMagnitude = sqrt(baselineX*baselineX + baselineY*baselineY + baselineZ*baselineZ);
  
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
  float impactAccel = sqrt(impactX*impactX + impactY*impactY + impactZ*impactZ);

  //compute gyro magnitude (total rotational velocity across all axes)
  float gyroMag = sqrt(gx*gx + gy*gy + gz*gz);

  //update peaks if this reading is the strongest so far in the window
  if (impactAccel > PeakSum) {
    PeakSum = impactAccel;
    PeakX = impactX;
    PeakY = impactY;
    PeakZ = impactZ;
  }

  if (gyroMag > PeakGyro) {
    PeakGyro = gyroMag;
    PeakGX = gx;
    PeakGY = gy;
    PeakGZ = gz;
  }

  //at the end of each 50ms window, report and check threshold
  if (millis() - lastWindowResetTime >= WINDOW_DURATION_MS) {
    
    //accel report
    Serial.print(" ACCEL | X: "); Serial.print(PeakX);
    Serial.print(" | Y: "); Serial.print(PeakY);
    Serial.print(" | Z: "); Serial.print(PeakZ);
    Serial.print(" | Sum: "); Serial.print(PeakSum);
    Serial.println(" m/s^2");

    //gyro report
    Serial.print("GYRO  | X: "); Serial.print(PeakGX);
    Serial.print(" | Y: "); Serial.print(PeakGY);
    Serial.print(" | Z: "); Serial.print(PeakGZ);
    Serial.print(" | Sum: "); Serial.print(PeakGyro);
    Serial.println(" deg/s");

    //threshold checks
    if (PeakSum > ACCEL_THRESHOLD) {
      triggerAlert("LINEAR", PeakSum, "m/s^2");
    }
    if (PeakGyro > GYRO_THRESHOLD) {
      triggerAlert("ROTATIONAL", PeakGyro, "deg/s");
    }

    //reset peaks and restart the window timer
    resetPeaks();
    lastWindowResetTime = millis();
  }
}