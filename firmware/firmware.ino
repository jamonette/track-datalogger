#include <Adafruit_GPS.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SD.h>
#include <Wire.h>

#define GpsSerial Serial1
#define GPS_SERIAL_RATE 9600

#define SD_SPI_CS_PIN 53
#define SD_FLUSH_INTERVAL 30000 // milliseconds

#define IMU_WRITE_INTERVAL 100 // milliseconds
#define LOCATION_MARKER_BUTTON_PIN 2
#define DEBOUNCE_DELAY 1000 // milliseconds

Adafruit_MPU6050 mpu;
Adafruit_GPS GPS(&GpsSerial);
File logFile;

int lastLocMarkerButtonEvent = 0;
bool locMarkerButtonIsPressed = false;
int lastImuWrite = 0;
int lastSdFlush = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) { }

  GpsSerial.begin(GPS_SERIAL_RATE);
  while (!GpsSerial) { }

  initSDCard();
  initGPS();
  initMPU();

  pinMode(LOCATION_MARKER_BUTTON_PIN, INPUT);
}

void loop() {
  int t = millis();

  GPS.read();
  if (GPS.newNMEAreceived()) {
    String gpsData = GPS.lastNMEA();
    Serial.print(gpsData);
    logFile.print(gpsData);
  }

  if (t - lastImuWrite > IMU_WRITE_INTERVAL) {
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);

    logFile.print("accel:");
    logFile.print(accel.acceleration.x);
    logFile.print(",");
    logFile.print(accel.acceleration.y);
    logFile.print(",");
    logFile.print(accel.acceleration.z);
    logFile.println("");

    Serial.print("accel:");
    Serial.print(accel.acceleration.x);
    Serial.print(",");
    Serial.print(accel.acceleration.y);
    Serial.print(",");
    Serial.print(accel.acceleration.z);
    Serial.println("");

    lastImuWrite = t;
  }

  if (digitalRead(LOCATION_MARKER_BUTTON_PIN) == HIGH) {

    // when button goes low->high, and we're outside of the
    // debounce window, log a start/finish marker event
    if (!locMarkerButtonIsPressed) {

      if (t - lastLocMarkerButtonEvent > DEBOUNCE_DELAY) {
        logFile.println("marklocation");
        Serial.println("LOCATION MARKER BUTTON PRESSED");
      }

      lastLocMarkerButtonEvent = t;
    }

    locMarkerButtonIsPressed = true;
  } else {
    locMarkerButtonIsPressed = false;
  }

  if (t - lastSdFlush > SD_FLUSH_INTERVAL) {
    lastSdFlush = t;
    logFile.flush();
  }
}

void initSDCard() {
  Serial.print("Initializing SD card...\t");

  pinMode(SD_SPI_CS_PIN, OUTPUT);
  if (!SD.begin(SD_SPI_CS_PIN)) {
    Serial.println("failed to initialize card. Halting.");
    enterErrorMode();
  }

  logFile = SD.open("datalog.dat", FILE_WRITE);
  lastSdFlush = millis();
  if (!logFile) {
    Serial.println("failed to open datalog file. Halting.");
    enterErrorMode();
  }

  Serial.println("done.");
}

void initGPS() {
  Serial.print("Initializing GPS...\t");
  GPS.begin(GPS_SERIAL_RATE);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
  Serial.println("done.");
}

void initMPU() {
  Serial.print("Initializing MPU6050...\t");

  if (!mpu.begin()) {
    Serial.println("failed.");
    enterErrorMode();
  }

  Serial.println("done");

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void enterErrorMode() {
  while(true) {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
  }
}
