#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL375.h>
#include <Adafruit_ISM330DHCX.h>
#include <NimBLEDevice.h>

//pins
const int LED_PIN = 21;                 //GPIO pin connected to the LED

//sensors
Adafruit_ADXL375 adxl(12345);
Adafruit_ISM330DHCX ism;                  //LSM6DS library supports ISM330DHCX              

//thresholds
const float ACCEL_THRESHOLD = 30;     //impact threshold in m/s^2
const float GYRO_THRESHOLD = 1500;      //rotational threshold in dps

//commands
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
float lastGyroMag = 0;  //previous sample's gyro magnitude, used to reject single-sample shock/glitch spikes

//window timing
unsigned long lastWindowResetTime = 0;          
const unsigned long WINDOW_DURATION_MS = 50;   //report peaks every 50ms

//rolling gravity baseline (tracks the gravity vector so it can be subtracted from live readings)
float baselineX = 0, baselineY = 0, baselineZ = 0;
const float ALPHA = 0.02; //how fast the baseline adapts (2% per reading (1000 readings per second))

//gyro drift offsets (calculated at startup, subtracted every loop)
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;

//BLE service + characteristic UUIDs (must match the HTML console exactly, don't change these unless you update both)
#define BLE_DEVICE_NAME    "ConcussionMonitor"
#define SERVICE_UUID       "ed2ce7c9-6308-447f-87ca-afc3fd8b23db"
#define ACCEL_CHAR_UUID    "aced99c9-0a3b-4a27-89ad-1f9138e7d810"
#define GYRO_CHAR_UUID     "4e65c5cf-f34e-4a09-82ba-04cfc23ac634"
#define ALERT_CHAR_UUID    "c9fd843f-c392-4eec-afbd-ecf46c16bb44"
#define COMMAND_CHAR_UUID  "3c50cfa9-9f5a-42e3-b08a-357c1efbf4f2"
#define STATUS_CHAR_UUID   "ba8a5476-129b-4470-87d6-814bda7c4c3c"

//BLE objects
NimBLEServer* pServer = nullptr;
NimBLECharacteristic* pAccelChar = nullptr;
NimBLECharacteristic* pGyroChar = nullptr;
NimBLECharacteristic* pAlertChar = nullptr;
NimBLECharacteristic* pCommandChar = nullptr;
NimBLECharacteristic* pStatusChar = nullptr;
bool deviceConnected = false;      //true = phone/computer currently connected over BLE

//resets all peak values to zero
void resetPeaks() {
  PeakX = PeakY = PeakZ = PeakSum = 0;
  PeakGX = PeakGY = PeakGZ = PeakGyro = 0;
}

//sends a status/feedback message to Serial and, if connected, to the website —
//keeps the Serial Monitor and web console showing identical command feedback
void sendStatus(String msg) {
  Serial.println(msg);
  if (deviceConnected) {
    pStatusChar->setValue(msg.c_str());
    pStatusChar->notify();
  }
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

  //BLE: push the alert immediately instead of waiting for the next window report
  if (deviceConnected) {
    String payload = reason + "," + String(value) + "," + unit;
    pAlertChar->setValue(payload.c_str());
    pAlertChar->notify();
  }
}

//samples both sensors for 2 seconds at rest to zero out offsets
//sets the gravity baseline and stores gyro drift offsets
//hold the device completely still during this window
void calibrateIMU() {
  Serial.println("Calibrating... keep sensor still.");

  double calGX = 0, calGY = 0, calGZ = 0;
  double calAX = 0, calAY = 0, calAZ = 0;
  int samples = 0;

  unsigned long calStart = millis();
  while (millis() - calStart < 2000) {

    //read accel
    sensors_event_t adxlEvent;
    adxl.getEvent(&adxlEvent);
    calAX += adxlEvent.acceleration.x;
    calAY += adxlEvent.acceleration.y;
    calAZ += adxlEvent.acceleration.z;

    //read gyro
    sensors_event_t accelEvent, gyroEvent, tempEvent; //ignore accel and temp; the library requires them to pass in order to also call for gyro
    ism.getEvent(&accelEvent, &gyroEvent, &tempEvent);
    //gyro comes in rad/s, convert to dps
    calGX += gyroEvent.gyro.x * (180.0 / PI);
    calGY += gyroEvent.gyro.y * (180.0 / PI);
    calGZ += gyroEvent.gyro.z * (180.0 / PI);

    samples++;
    delay(2);
  }

  //seed the accel baseline directly from calibration averages
  baselineX = calAX / samples;
  baselineY = calAY / samples;
  baselineZ = calAZ / samples;

  //store gyro averages as offsets to subtract every loop
  gyroOffsetX = calGX / samples;
  gyroOffsetY = calGY / samples;
  gyroOffsetZ = calGZ / samples;

  resetPeaks();
  lastWindowResetTime = millis();

  sendStatus("Calibration completed.");
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
      sendStatus("Already on. Use 'clear' to resume monitoring or 'stop' to turn off.");
      return;
    }
    systemRunning = true;
    thresholdActive = true;
    resetPeaks();
    lastWindowResetTime = millis(); //starts the first window
    digitalWrite(LED_PIN, LOW);
    sendStatus("System on. Monitoring started.");
  }
  else if (input == STOP_PHRASE) {
    if (!systemRunning) {
      //ignore if already off
      sendStatus("System is already off.");
      return;
    }
    systemRunning = false;
    thresholdActive = false;
    digitalWrite(LED_PIN, LOW);
    sendStatus("System off.");
  }
  else if (input == RESTART_PHRASE) {
    if (!systemRunning) {
      //can't resume if the device isn't on
      sendStatus("System is off. Use 'start' first.");
      return;
    }
    if (thresholdActive) {
      //already monitoring, nothing to resume
      sendStatus("Already monitoring.");
      return;
    }
    //system is on but paused after an alert, so resume monitoring
    resetPeaks();
    lastWindowResetTime = millis();
    digitalWrite(LED_PIN, LOW);
    calibrateIMU();
    thresholdActive = true;
  }
  else if (input == CAL_PHRASE) {
    if (systemRunning) {
      //system must be off to calibrate, otherwise IMU will continue to send readings
      sendStatus("Stop the system before calibrating. Use 'stop' first.");
      return;
    }
    calibrateIMU();
  }
  else {
    //unknown commands
    sendStatus("Unknown command: '" + input + "'. Use start, stop, clear, or calibrate");
  }
}

//BLE: fires when a phone/computer connects or disconnects
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
    deviceConnected = true;
    Serial.println("BLE: connected.");
  }
  void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
    deviceConnected = false;
    Serial.println("BLE: disconnected, restarting advertising.");
    NimBLEDevice::getAdvertising()->start();
  }
};

//BLE: lets a connected phone/computer send start, stop, clear, calibrate
//just like typing them into the Serial Monitor — routes straight into handleCommand()
class CommandCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override {
    String input = String(pChar->getValue().c_str());
    Serial.print("BLE command received: ");
    Serial.println(input);
    handleCommand(input);
  }
};

//sets up the BLE service and starts advertising
void setupBLE() {
  NimBLEDevice::init(BLE_DEVICE_NAME);

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  NimBLEService* pService = pServer->createService(SERVICE_UUID);

  pAccelChar = pService->createCharacteristic(
    ACCEL_CHAR_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
  );
  pGyroChar = pService->createCharacteristic(
    GYRO_CHAR_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
  );
  pAlertChar = pService->createCharacteristic(
    ALERT_CHAR_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
  );
  pCommandChar = pService->createCharacteristic(
    COMMAND_CHAR_UUID,
    NIMBLE_PROPERTY::WRITE
  );
  pStatusChar = pService->createCharacteristic(
    STATUS_CHAR_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
  );
  pCommandChar->setCallbacks(new CommandCallbacks());

  pService->start();

  NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();

  Serial.println("BLE advertising started as \"ConcussionDetector\".");
}

//setup
void setup() {
  pinMode(LED_PIN, OUTPUT);
  
  Serial.begin(115200);
  delay(2000);
  while (!Serial) delay(10); //wait for serial monitor to open

  Wire.begin(); //start I2C bus
  delay(100);   //give sensor time to power up


  //initialize ADXL375
  Serial.println("Initializing ADXL375...");
  if (!adxl.begin()) {
    Serial.println("ADXL375 not found. Check wiring.");
    while (1) delay(10);
  }
  adxl.setDataRate(ADXL343_DATARATE_800_HZ);
  Serial.println("ADXL375 connected.");

  //initialize ISM330DHCX
  Serial.println("Initializing ISM330DHCX...");
  if (!ism.begin_I2C()) {
    Serial.println("ISM330DHCX not found. Check wiring.");
    while (1) delay (10);
  }
  ism.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
  ism.setGyroDataRate(LSM6DS_RATE_833_HZ);
  Serial.println("ISM330DHCX connected.");
  Serial.println("Please wait two seconds while sensors calibrate...");

  digitalWrite(LED_PIN, LOW);
  calibrateIMU();  

  setupBLE();
}

//loop (actual process)
void loop() {
  //check for incoming serial commands each cycle
  if (Serial.available() > 0) {
    handleCommand(Serial.readStringUntil('\n'));
  }

  //skip IMU reading if system is off or paused after alert
  if (!systemRunning || !thresholdActive) return;

  //read ADXL375
  sensors_event_t adxlEvent;
  if (!adxl.getEvent(&adxlEvent)) return;
  float ax = adxlEvent.acceleration.x;
  float ay = adxlEvent.acceleration.y;
  float az = adxlEvent.acceleration.z;

  //read ISM330DHCX gyro; convert rad/s -> dps
  sensors_event_t ismAccelEvent, ismGyroEvent, ismTempEvent;
  if (!ism.getEvent (&ismAccelEvent, &ismGyroEvent, &ismTempEvent)) return;
  float gx = (ismGyroEvent.gyro.x * (180.0 / PI)) - gyroOffsetX;
  float gy = (ismGyroEvent.gyro.y * (180.0 / PI)) - gyroOffsetY;
  float gz = (ismGyroEvent.gyro.z * (180.0 / PI)) - gyroOffsetZ;

  //compute magnitude of the current gravity baseline
  float baselineMagnitude = sqrt(baselineX*baselineX + baselineY*baselineY + baselineZ*baselineZ);
  //compute total acceleration magnitude of live reading
  float totalAccel = sqrt(ax*ax + ay*ay + az*az);

  //only update baseline during quiet moments (no impact happening)
  if (totalAccel - baselineMagnitude < 2.0) {
    baselineX = ALPHA * ax + (1 - ALPHA) * baselineX;
    baselineY = ALPHA * ay + (1 - ALPHA) * baselineY;
    baselineZ = ALPHA * az + (1 - ALPHA) * baselineZ; 
  }

  //subtract gravity baseline to isolate impact
  float impactX = ax - baselineX;
  float impactY = ay - baselineY;
  float impactZ = az - baselineZ;
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

    //BLE: push the same window report, only if someone's actually connected
    if (deviceConnected) {
      String accelPayload = String(PeakX, 2) + "," + String(PeakY, 2) + "," + String(PeakZ, 2) + "," + String(PeakSum, 2);
      pAccelChar->setValue(accelPayload.c_str());
      pAccelChar->notify();

      String gyroPayload = String(PeakGX, 2) + "," + String(PeakGY, 2) + "," + String(PeakGZ, 2) + "," + String(PeakGyro, 2);
      pGyroChar->setValue(gyroPayload.c_str());
      pGyroChar->notify();
    }

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