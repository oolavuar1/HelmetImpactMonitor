#include <Wire.h>

const int LED_PIN = 21;

const int MPU_ADDR = 0x68; 

const float ACCEL_THRESHOLD = 15.0; 

void setup() {
  pinMode(LED_PIN, OUTPUT);

  Serial.begin(115200);
  while (!Serial) delay(10);

  delay(10000);

  Serial.println("Initializing MPU6050 via Direct I2C...");

  Wire.begin();
  delay(100);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0); 
  byte error = Wire.endTransmission();

  if (error == 0) {
    Serial.println("SUCCESS: Direct connection established with MPU6050!");
  } 
  else {
    Serial.print("FAILED: Direct connection error code: ");
    Serial.println(error);
    while (1) delay(10);
  }

  digitalWrite(LED_PIN, LOW);
}

void loop() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  int16_t rawX = (Wire.read() << 8) | Wire.read();
  int16_t rawY = (Wire.read() << 8) | Wire.read();
  int16_t rawZ = (Wire.read() << 8) | Wire.read();

  float x = (rawX / 16384.0) * 9.8;
  float y = (rawY / 16384.0) * 9.8;
  float z = (rawZ / 16384.0) * 9.8;

  float totalAccel = sqrt((x * x) + (y * y) + (z * z));

  float impactAccel = totalAccel - 9.8;

  if (impactAccel < 0) impactAccel = 0;

  Serial.print("X: "); Serial.print(x);
  Serial.print(" | Y: "); Serial.print(y);
  Serial.print(" | Z: "); Serial.print(z);
  Serial.print(" | Total: "); Serial.println(impactAccel);


  if (impactAccel > ACCEL_THRESHOLD) {
    Serial.println("=========================================");
    Serial.println("Direct Mode: THRESHOLD EXCEEDED!");
    Serial.println("=========================================");
    digitalWrite(LED_PIN, HIGH);
    delay(2000);
    digitalWrite(LED_PIN, LOW);
  }

  delay(100);
}