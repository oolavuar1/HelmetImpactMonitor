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
const float ACCEL_THRESHOLD = 784;     //impact threshold in m/s^2
const float GYRO_THRESHOLD = 1000;      //rotational threshold in dps

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

#define TEST_CHAR_UUID     "8d20860e-d033-474a-9703-bd1da03201a0"
NimBLECharacteristic* pTestChar = nullptr;

//testing commands
const String ARM_PHRASE = "arm";          //arms an impact capture (system must be off)
const String DISARM_PHRASE = "disarm";    //cancels an armed capture
const String GRAVITY_PHRASE = "gravity";  //reads raw gravity for the scale check (system must be off)

//testing state
volatile bool captureArmed = false;
volatile bool gravityPending = false;   //runs in loop() instead of the BLE callback so it doesn't block the BLE stack
unsigned long loopCount = 0;            //loops per window, sent to the console to measure sample rate

//impact capture: keeps a rolling buffer before the hit, then records after it
const int CAP_PRE = 64;                 //samples kept from before the impact
const int CAP_POST = 192;               //samples recorded after the trigger
const float CAP_TRIGGER = 49.0;         //raw reading (m/s^2, ~5g) that counts as an impact
struct CapSample { uint32_t t; float x, y, z; };
CapSample preRing[CAP_PRE];
CapSample capOut[CAP_PRE + CAP_POST];
int capHead = 0, capFilled = 0, capCount = 0;
bool capTriggered = false;

//rotation check (test 5)
const String ROTARM_PHRASE = "rotarm";    //zeroes the gyro then waits for a turn (system has to be off)
const String ROTSTOP_PHRASE = "rotstop";  //stops it by hand, also used for the still test
const float ROT_MOVING_DPS = 10.0;        //anything faster than this counts as turning
const unsigned long ROT_SETTLE_MS = 500;  //stops after being under the turning speed for this long
const unsigned long ROT_MAX_MS = 20000;   //max time for one run so it doesn't go forever
const unsigned long ROT_LOOKBACK_US = 300000;  //adds the last 0.3s before the turn so the speed up gets counted
const int ROT_RING = 1024;                     //how many slices to save while waiting

volatile bool rotZeroPending = false;     //zeroing happens in loop so it doesn't block BLE
volatile bool rotActive = false;          //true = rotation check is running
volatile bool rotStopPending = false;     //true = stop on the next loop
volatile bool rotManualStop = false;      //true = stopped by hand (lets the still test send data)

float rotBiasX = 0, rotBiasY = 0, rotBiasZ = 0;         //gyro zero, measured right before each turn
double rotAngX = 0, rotAngY = 0, rotAngZ = 0;           //whole turn angle for each axis (deg)
double rotMovAngX = 0, rotMovAngY = 0, rotMovAngZ = 0;  //threshold method angle, only adds while turning
double rotAllX = 0, rotAllY = 0, rotAllZ = 0;           //everything since arm, only for the still test
float rotPrevX = 0, rotPrevY = 0, rotPrevZ = 0;         //last reading, used to make the slices
float rotPeak = 0;                        //fastest speed in the run
uint32_t rotPrevT = 0;                    //time of the last reading (us)
unsigned long rotStartMs = 0, rotSamples = 0, rotMovSamples = 0, rotLastTurningMs = 0;
bool rotFirst = true;                     //first reading doesn't have a slice yet
bool rotStarted = false;                  //true = helmet went over the turning speed

//saves recent slices while waiting for the turn to start
float ringX[ROT_RING], ringY[ROT_RING], ringZ[ROT_RING];
uint32_t ringDt[ROT_RING];                //how long each slice is (us)
int ringHead = 0, ringCount = 0;

//resets all peak values to zero
void resetPeaks() {
  PeakX = PeakY = PeakZ = PeakSum = 0;
  PeakGX = PeakGY = PeakGZ = PeakGyro = 0;
  loopCount = 0;
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

//averages raw accelerometer readings for half a second and sends them to the console
void runGravityCheck() {
  double sx = 0, sy = 0, sz = 0;
  int n = 0;
  unsigned long start = millis();
  while (millis() - start < 500) {
    sensors_event_t e;
    if (adxl.getEvent(&e)) {
      sx += e.acceleration.x; sy += e.acceleration.y; sz += e.acceleration.z;
      n++;
    }
    delay(2);
  }
  if (n == 0) return;
  String msg = "GRAV," + String(sx / n, 3) + "," + String(sy / n, 3) + "," + String(sz / n, 3);
  Serial.println(msg);
  if (deviceConnected) {
    pTestChar->setValue(msg.c_str());
    pTestChar->notify();
  }
}

//sends the captured impact to the console one sample per packet
void sendCapture() {
  sendStatus("Impact captured. Sending " + String(capCount) + " samples...");
  if (!deviceConnected) return;
  uint32_t t0 = capOut[0].t;
  String head = "CAP_BEGIN," + String(capCount) + "," + String(baselineX, 3) + "," + String(baselineY, 3) + "," + String(baselineZ, 3);
  pTestChar->setValue(head.c_str());
  pTestChar->notify();
  delay(10);
  for (int i = 0; i < capCount; i++) {
    String line = "C," + String(capOut[i].t - t0) + "," + String(capOut[i].x, 2) + "," + String(capOut[i].y, 2) + "," + String(capOut[i].z, 2);
    pTestChar->setValue(line.c_str());
    pTestChar->notify();
    delay(6);   //gives the BLE stack time to send each packet
  }
  pTestChar->setValue("CAP_END");
  pTestChar->notify();
  sendStatus("Capture sent.");
}

//reads the accelerometer as fast as possible while armed; called from loop()
void runCapture() {
  sensors_event_t e;
  if (!adxl.getEvent(&e)) return;
  CapSample s = { (uint32_t)micros(), e.acceleration.x, e.acceleration.y, e.acceleration.z };

  if (!capTriggered) {
    //before the hit: keep overwriting a rolling buffer
    preRing[capHead] = s;
    capHead = (capHead + 1) % CAP_PRE;
    if (capFilled < CAP_PRE) capFilled++;
    if (sqrt(s.x*s.x + s.y*s.y + s.z*s.z) > CAP_TRIGGER) {
      //hit detected: copy the rolling buffer out oldest-first, then keep recording
      int start = (capHead - capFilled + CAP_PRE) % CAP_PRE;
      for (int i = 0; i < capFilled; i++) capOut[i] = preRing[(start + i) % CAP_PRE];
      capCount = capFilled;
      capTriggered = true;
    }
  } else {
    capOut[capCount++] = s;
    if (capCount >= capFilled + CAP_POST) {
      captureArmed = false;
      capTriggered = false;
      sendCapture();
    }
  }
}

//averages the gyro for half a second while still to get a new zero, then waits for a turn
void runRotZero() {
  sendStatus("Zeroing gyro... keep the helmet still.");
  double sx = 0, sy = 0, sz = 0;
  int n = 0;
  unsigned long start = millis();
  while (millis() - start < 500) {
    sensors_event_t a, g, t;
    if (ism.getEvent(&a, &g, &t)) {
      //gyro comes in rad/s, convert to dps
      sx += g.gyro.x * (180.0 / PI);
      sy += g.gyro.y * (180.0 / PI);
      sz += g.gyro.z * (180.0 / PI);
      n++;
    }
    delay(2);
  }
  rotZeroPending = false;
  if (n == 0) { sendStatus("Gyro read failed."); return; }
  rotBiasX = sx / n; rotBiasY = sy / n; rotBiasZ = sz / n;

  //reset everything for the new run
  rotAngX = rotAngY = rotAngZ = 0;
  rotMovAngX = rotMovAngY = rotMovAngZ = 0;
  rotAllX = rotAllY = rotAllZ = 0;
  rotPeak = 0; rotSamples = 0; rotMovSamples = 0;
  ringHead = 0; ringCount = 0;
  rotFirst = true;
  rotStarted = false;
  rotManualStop = false;
  rotStartMs = millis();
  rotActive = true;
  sendStatus("Rotation armed. Turn it when ready; it stops on its own.");
}

//adds up speed x time for each axis (trapezoid rule), runs every loop while active
//before the turn starts, slices just get saved. once it starts, the last 0.3s gets added in
//also keeps a second total that only counts readings over the turning speed (threshold method)
//stops by itself once the helmet settles
void runRotation() {
  sensors_event_t a, g, t;
  if (!ism.getEvent(&a, &g, &t)) return;
  uint32_t now = micros();

  //convert to dps and subtract the zero
  float gx = g.gyro.x * (180.0 / PI) - rotBiasX;
  float gy = g.gyro.y * (180.0 / PI) - rotBiasY;
  float gz = g.gyro.z * (180.0 / PI) - rotBiasZ;
  float mag = sqrt(gx*gx + gy*gy + gz*gz);
  bool turning = mag > ROT_MOVING_DPS;

  //turn just started, so add the last 0.3s from before it so the speed up counts
  if (turning && !rotStarted) {
    rotStarted = true;
    uint32_t back = 0;
    for (int i = 0; i < ringCount && back < ROT_LOOKBACK_US; i++) {
      int k = (ringHead - 1 - i + ROT_RING) % ROT_RING;   //goes newest to oldest
      rotAngX += ringX[k]; rotAngY += ringY[k]; rotAngZ += ringZ[k];
      back += ringDt[k];
    }
    sendStatus("Turning.");
  }

  if (!rotFirst) {
    //slice = average of the last two speeds x time between them
    uint32_t dtUs = now - rotPrevT;
    double dt = dtUs / 1000000.0;
    double sx = (gx + rotPrevX) * 0.5 * dt;
    double sy = (gy + rotPrevY) * 0.5 * dt;
    double sz = (gz + rotPrevZ) * 0.5 * dt;
    rotAllX += sx; rotAllY += sy; rotAllZ += sz;   //still test total

    if (rotStarted) {
      rotAngX += sx; rotAngY += sy; rotAngZ += sz;                            //whole turn
      if (turning) { rotMovAngX += sx; rotMovAngY += sy; rotMovAngZ += sz; }  //threshold method, only while turning
    } else {
      //still waiting, so save the slice instead of adding it
      ringX[ringHead] = sx; ringY[ringHead] = sy; ringZ[ringHead] = sz; ringDt[ringHead] = dtUs;
      ringHead = (ringHead + 1) % ROT_RING;
      if (ringCount < ROT_RING) ringCount++;
    }
  }
  if (turning) {
    rotMovSamples++;
    rotLastTurningMs = millis();
  }

  rotFirst = false;
  rotPrevX = gx; rotPrevY = gy; rotPrevZ = gz; rotPrevT = now;
  if (mag > rotPeak) rotPeak = mag;
  rotSamples++;

  //auto stop: the turn started and it's been under the turning speed long enough
  if (rotStarted && millis() - rotLastTurningMs > ROT_SETTLE_MS) rotStopPending = true;
  //max time so a forgotten run doesn't go forever
  if (millis() - rotStartMs > ROT_MAX_MS) rotStopPending = true;
}

//stops the check and sends both totals, peak speed, time, samples, and how much the zero moved since calibration
void finishRotation() {
  rotActive = false;
  rotStopPending = false;
  if (!rotStarted && !rotManualStop) {
    sendStatus("No turn detected. Arm again.");
    return;
  }
  //still test (stopped by hand with no turn), so send everything since arm instead
  if (!rotStarted) { rotAngX = rotAllX; rotAngY = rotAllY; rotAngZ = rotAllZ; }

  float secs = (millis() - rotStartMs) / 1000.0;
  float drift = sqrt(pow(rotBiasX - gyroOffsetX, 2) + pow(rotBiasY - gyroOffsetY, 2) + pow(rotBiasZ - gyroOffsetZ, 2));
  String msg = "ROT," + String(rotAngX, 2) + "," + String(rotAngY, 2) + "," + String(rotAngZ, 2) + ","
             + String(rotPeak, 1) + "," + String(secs, 2) + "," + String(rotSamples) + "," + String(drift, 3) + ","
             + String(rotMovSamples) + "," + String(rotMovAngX, 2) + "," + String(rotMovAngY, 2) + "," + String(rotMovAngZ, 2) + ","
             + String(ROT_MOVING_DPS, 0);
  Serial.println(msg);
  if (deviceConnected) {
    pTestChar->setValue(msg.c_str());
    pTestChar->notify();
  }
  sendStatus("Rotation stopped.");
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
    captureArmed = false;
    systemRunning = true;
    thresholdActive = true;
    //starting the system cancels a rotation check
    rotActive = false;
    rotZeroPending = false;
    rotStopPending = false;
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
  else if (input == ARM_PHRASE) {
    if (systemRunning) {
      sendStatus("Stop the system before arming a capture. Use 'stop' first.");
      return;
    }
    capHead = 0; capFilled = 0; capCount = 0; capTriggered = false;
    captureArmed = true;   //set last so loop() never sees a half-reset buffer
    sendStatus("Capture armed. Drop when ready.");
  }
  else if (input == DISARM_PHRASE) {
    captureArmed = false;
    capTriggered = false;
    sendStatus("Capture cancelled.");
  }
  else if (input == GRAVITY_PHRASE) {
    if (systemRunning) {
      sendStatus("Stop the system before a gravity check. Use 'stop' first.");
      return;
    }
    gravityPending = true;
  }
  else if (input == ROTARM_PHRASE) {
    if (systemRunning) {
      //system has to be off so it doesn't mix with monitoring
      sendStatus("Stop the system before a rotation check. Use 'stop' first.");
      return;
    }
    rotActive = false;
    rotZeroPending = true;
  }
  else if (input == ROTSTOP_PHRASE) {
    if (!rotActive) {
      //nothing to stop
      sendStatus("No rotation check running.");
      return;
    }
    rotManualStop = true;
    rotStopPending = true;
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
  NimBLEDevice::setMTU(185);   //capture packets are longer than the 20-byte default

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
  pTestChar = pService->createCharacteristic(
  TEST_CHAR_UUID,
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
  Wire.setClock(400000);
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

  //testing tools run instead of monitoring while active
  if (gravityPending) { gravityPending = false; runGravityCheck(); return; }
  if (captureArmed) { runCapture(); return; }
  if (rotZeroPending) { runRotZero(); return; }
  if (rotStopPending) { finishRotation(); return; }
  if (rotActive) { runRotation(); return; }

  //skip IMU reading if system is off or paused after alert
  if (!systemRunning || !thresholdActive) return;

  loopCount++;

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
      String accelPayload = String(PeakX, 2) + "," + String(PeakY, 2) + "," + String(PeakZ, 2) + "," + String(PeakSum, 2) + "," + String(loopCount) + "," + String(millis() - lastWindowResetTime);
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