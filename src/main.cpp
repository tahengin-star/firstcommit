#include <Arduino.h>
#include <ESP32Servo.h> // servo support for ESP32
#include <ESP32PWM.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_GC9A01A.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <ThreeWire.h>
#include <RtcDS1302.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <stdlib.h>
#include <math.h>

// Display mode enum
enum DisplayMode {
  MODE_BOOT,  // Boot screen with animated emoji
  MODE_INFO   // Info screen with S1/S2/Battery/BLE
};

enum InfoPage {
  INFO_PAGE_STATUS = 0,
  INFO_PAGE_SENSORS = 1,
  INFO_PAGE_IMU = 2,
  INFO_PAGE_CPU = 3,
  INFO_PAGE_MAX
};

// GC9A01 round TFT display object (240x240, SPI)
const int tftCsPin = 5;
const int tftDcPin = 16;
const int tftRstPin = 17;
const int tftMosiPin = 13;
const int tftSclkPin = 14;
Adafruit_GC9A01A display(tftCsPin, tftDcPin, tftMosiPin, tftSclkPin, tftRstPin);
const int displayWidthPx = 240;
const int displayHeightPx = 240;
const int uiPanelWidthPx = 128;
const int uiPanelHeightPx = 64;
const int uiPanelX = (displayWidthPx - uiPanelWidthPx) / 2;
const int uiPanelY = (displayHeightPx - uiPanelHeightPx) / 2;
const bool displayEnabled = true;
Adafruit_MPU6050 mpu;
Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp;
const int rtcIoPin = 26;
const int rtcSclkPin = 25;
const int rtcCePin = 27;
ThreeWire rtcWire(rtcIoPin, rtcSclkPin, rtcCePin);
RtcDS1302<ThreeWire> rtc(rtcWire);

// BLE UART-like service (matches web_controller.html UUIDs)
BLECharacteristic *commandCharacteristic = nullptr;

// put function declarations here:
void updateOLED();
void drawBootScreen();
void drawStaticFrame();
void drawRotatingFrame();
void drawInfoScreen();
void drawTopBarDateTime();
void runOledStartupTest();
void drawHappyFace();
void drawAngryFace();
void processCommand(char cmd);
void processPacket(const std::string &packet);
void setServoAnglesFromNormalized(int xNorm, int yNorm);
void updateJoystickInputStats(int rawXNorm, int rawYNorm);
void reportJoystickInputStats();
void setMobileDateTime(int year, int month, int day, int hour, int minute, int second);
int toServo1OutputCommand(int logicalCommand);
void writeServo1Command(int logicalCommand);
void writeServo2Command(int logicalCommand);
void ensureServosAttached();
bool attachServos();
int moveToward(int currentValue, int targetValue, int maxStep);
int clampServoCommand(int command);
int applyDeadband(int command, int deadbandMargin);
int quantizeCommand(int command, int stepSize);
int commandToPercent(int command);
void initSensors();
bool readSensors();
void initRTC();
bool readRTC();
void scanI2CBus();
void scanI2CBusPeriodic();
bool readMpuWhoAmI(uint8_t address, uint8_t &whoAmI);
bool readMpuRawSample();
void sendTelemetryPackets();
void sendTelemetryLine(const char *line);
void setPumpRunning(bool enabled);
void displayTaskFunc(void *param);

// Boot screen animation state
unsigned long lastBlinkTime = 0;
const unsigned long blinkIntervalMs = 300;  // 0.3 second
bool eyeOpen = true;

unsigned long lastAnimFrameTime = 0;
const unsigned long animFrameIntervalMs = 80;
uint8_t framePhase = 0;
const uint8_t framePhaseMax = 64;

std::string rxBuffer;  // BLE packet buffer

class CommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    std::string rx = pCharacteristic->getValue();
    for (size_t i = 0; i < rx.size(); i++) {
      char incoming = rx[i];
      if (incoming == '\n' || incoming == '\r') {
        if (!rxBuffer.empty()) {
          processPacket(rxBuffer);
          rxBuffer.clear();
        }
      } else {
        rxBuffer.push_back(incoming);
        if (rxBuffer.size() > 64) {
          rxBuffer.clear();
        }
      }
    }
  }
};

Servo myServo1;
Servo myServo2;
const int servo1Pin = 18;
const int servo2Pin = 19;
const int servoPulseMinUs = 500;
const int servoPulseMaxUs = 2500;
const int servoMinCommand = 0;
const int servoMaxCommand = 180;
const int servoNeutralCommand = 90;
const int servo1Deadband = 2;
const int servo2Deadband = 2;
const int servoHomeDeadband = 5;
const int servo1CommandHysteresis = 1;
const int servo2CommandHysteresis = 1;
const int servo1SlewStepActiveDeg = 5;
const int servo2SlewStepActiveDeg = 5;
const int servoSlewStepIdleDeg = 4;
const unsigned long servoSlewIntervalMs = 8;
const int servo1SoftMinCommand = 8;
const int servo1SoftMaxCommand = 172;
const int servo1OutputMinUs = 600;
const int servo1OutputMaxUs = 2400;
const int servo1CenterTrimUs = 0;
const int servo2SoftMinCommand = 8;
const int servo2SoftMaxCommand = 172;
const int servo2OutputMinUs = 600;
const int servo2OutputMaxUs = 2400;
const int servo2CenterTrimUs = 0;
const int manualDriveOffset = 55;
const int sweepRangeOffset = 70;
int currentAngle1 = servoNeutralCommand;
int currentAngle2 = servoNeutralCommand;
int lastWrittenAngle1 = servoNeutralCommand;
int lastWrittenAngle2 = servoNeutralCommand;
int lastServo1OutputUs = 1500;
int lastServo2OutputUs = 1500;
bool servosAttached = false;
unsigned long lastServoCommandMs = 0;
unsigned long lastServoPwmDebugMs = 0;
const unsigned long servoPwmDebugIntervalMs = 200;
const bool servoPwmDebugEnabled = true;
const bool servoAutoDetachEnabled = false;
const unsigned long servoAutoDetachMs = 250;
int joystickTargetAngle1 = servoNeutralCommand;
int joystickTargetAngle2 = servoNeutralCommand;
const int joystickCommandStepDeg = 1;
const int joystickNormDeadzone = 1;
const int joystickNormFilterAlphaNumerator = 1;
const int joystickNormFilterAlphaDenominator = 1;
const int joystickNormMaxStepPerPacket = 100;
const int joystickTargetStepPerPacketDeg = 12;
const bool joystickInputStatsEnabled = true;
const unsigned long joystickInputStatsIntervalMs = 1000;
bool joystickActive = false;
unsigned long joystickLastUpdateMs = 0;
const unsigned long joystickActiveTimeoutMs = 350;
int joystickRawXNorm = 0;
int joystickRawYNorm = 0;
int joystickFilteredXNorm = 0;
int joystickFilteredYNorm = 0;
unsigned long joystickStatsWindowStartMs = 0;
unsigned long joystickStatsLastPacketMs = 0;
bool joystickStatsHasPrevSample = false;
int joystickStatsPrevXNorm = 0;
int joystickStatsPrevYNorm = 0;
unsigned int joystickStatsPacketCount = 0;
unsigned long joystickStatsDtSumMs = 0;
unsigned long joystickStatsDtMinMs = 0;
unsigned long joystickStatsDtMaxMs = 0;
unsigned long joystickStatsAbsDeltaXSum = 0;
unsigned long joystickStatsAbsDeltaYSum = 0;
int joystickStatsDeltaXMax = 0;
int joystickStatsDeltaYMax = 0;
bool oledReady = false;
bool autoLevelEnabled = true;
bool autoLevelReferenceCaptured = false;
unsigned long autoLevelCaptureStartMs = 0;
const unsigned long autoLevelCaptureDelayMs = 1500;
float autoLevelRollRefDeg = 0.0f;
float autoLevelPitchRefDeg = 0.0f;
const float autoLevelDeadbandDeg = 1.5f;
const float autoLevelGainS1 = 1.1f;
const float autoLevelGainS2 = 1.1f;
const int autoLevelMaxCompDeg = 30;

// Water pump motor
const int pumpPin = 23;  // GPIO23 -> relay IN1
const bool pumpRelayActiveLow = true;
bool pumpRunning = false;

// Servo sweep state
volatile bool servo1Sweeping = false;
int sweepDirection = 1;  // 1 for increasing, -1 for decreasing

// Display system
DisplayMode currentMode = MODE_INFO;
InfoPage currentInfoPage = INFO_PAGE_STATUS;
bool bluetoothConnected = false;
volatile bool displayDirty = true;
TaskHandle_t displayTaskHandle = nullptr;
const int statusLedPin = 2;  // ESP32 onboard LED (typical on dev boards)
bool statusLedState = false;
unsigned long lastStatusLedBlinkMs = 0;
const unsigned long statusLedBlinkIntervalMs = 350;
int batteryVoltage = 0;
const int batteryPin = 34;  // ADC pin for battery voltage (VP pin)
bool mpuReady = false;
bool mpuFallbackMode = false;
uint8_t mpuWhoAmI = 0;
bool ahtReady = false;
bool bmpReady = false;
bool rtcReady = false;
uint8_t ahtReadFailCount = 0;
uint8_t bmpReadFailCount = 0;
const uint8_t i2cSensorFailThreshold = 5;
uint8_t mpuI2CAddress = 0;
uint8_t bmpI2CAddress = 0;
float accelX = 0.0f;
float accelY = 0.0f;
float accelZ = 0.0f;
float gyroX = 0.0f;
float gyroY = 0.0f;
float gyroZ = 0.0f;
float mpuTempC = 0.0f;
float rollDeg = 0.0f;
float pitchDeg = 0.0f;
bool imuAnglesInitialized = false;
unsigned long lastImuUpdateMs = 0;
float ahtTempC = 0.0f;
float ahtHum = 0.0f;
float bmpTempC = 0.0f;
float bmpPressureHPa = 0.0f;
uint16_t rtcYear = 0;
uint8_t rtcMonth = 0;
uint8_t rtcDay = 0;
uint8_t rtcHour = 0;
uint8_t rtcMinute = 0;
uint8_t rtcSecond = 0;
bool mobileTimeValid = false;
bool mobileTimeUpdated = false;
unsigned long mobileTimeLastSyncMs = 0;
uint16_t mobileYear = 0;
uint8_t mobileMonth = 0;
uint8_t mobileDay = 0;
uint8_t mobileHour = 0;
uint8_t mobileMinute = 0;
uint8_t mobileSecond = 0;
unsigned long lastTelemetryMs = 0;
const unsigned long telemetryIntervalMs = 400;
const bool i2cPeriodicScanEnabled = false;
const unsigned long i2cPeriodicScanIntervalMs = 2000;
unsigned long lastI2CPeriodicScanMs = 0;
const bool oledStartupTestEnabled = false;
const bool sensorSerialLogEnabled = false;
const bool reduceI2CWhileServoActive = true;

// CPU load measurement
volatile unsigned long cpuIdleCountCore0 = 0;
volatile unsigned long cpuIdleCountCore1 = 0;
float cpuLoadCore0Pct = 0.0f;
float cpuLoadCore1Pct = 0.0f;
unsigned long cpuCalibIdlePerSec = 0;  // idle ticks in 1s with no load

// State for button press
volatile bool leftPressed = false;
volatile bool rightPressed = false;
volatile bool forwardPressed = false;
volatile bool backPressed = false;
const int stepAngle = 8;
const int minAngle = servoMinCommand;
const int maxAngle = servoMaxCommand;

// CPU idle counter tasks (lowest priority, one per core)
void cpuIdleCountTask0(void *param) {
  for (;;) {
    cpuIdleCountCore0++;
    vTaskDelay(1);  // yield to other tasks, count increments per tick
  }
}
void cpuIdleCountTask1(void *param) {
  for (;;) {
    cpuIdleCountCore1++;
    vTaskDelay(1);
  }
}

void setup() {
  Serial.begin(115200);

  // Start CPU idle counter tasks (lowest priority) on each core
  xTaskCreatePinnedToCore(cpuIdleCountTask0, "IdleCnt0", 1024, nullptr, 0, nullptr, 0);
  xTaskCreatePinnedToCore(cpuIdleCountTask1, "IdleCnt1", 1024, nullptr, 0, nullptr, 1);
  // Calibrate: measure idle ticks in 200ms with minimal load
  cpuIdleCountCore0 = 0;
  cpuIdleCountCore1 = 0;
  vTaskDelay(pdMS_TO_TICKS(200));
  cpuCalibIdlePerSec = cpuIdleCountCore1 * 5;  // extrapolate to 1 second
  if (cpuCalibIdlePerSec == 0) cpuCalibIdlePerSec = 1;
  cpuIdleCountCore0 = 0;
  cpuIdleCountCore1 = 0;
  Serial.printf("CPU idle calibration: %lu ticks/sec\n", cpuCalibIdlePerSec);

  BLEDevice::init("ESP32_Servo");
  BLEServer *server = BLEDevice::createServer();
  BLEService *service = server->createService(BLEUUID((uint16_t)0xFFE0));
  commandCharacteristic = service->createCharacteristic(
    BLEUUID((uint16_t)0xFFE1),
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR |
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  commandCharacteristic->addDescriptor(new BLE2902());
  commandCharacteristic->setCallbacks(new CommandCallbacks());
  service->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(BLEUUID((uint16_t)0xFFE0));
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);
  advertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("BLE advertising started: ESP32_Servo (FFE0/FFE1)");

  // I2C bus init (sensors/RTC)
  Wire.begin(21, 22); // SDA, SCL
  Wire.setClock(100000);
  Wire.setTimeOut(25);
  scanI2CBus();

  // Display runs as separate FreeRTOS task on Core 0
  if (displayEnabled) {
    xTaskCreatePinnedToCore(displayTaskFunc, "Display", 8192, nullptr, 1, &displayTaskHandle, 0);
    Serial.println("Display task created on Core 0");
  } else {
    oledReady = false;
    Serial.println("Display disabled (servo-only mode)");
  }

  initSensors();
  initRTC();

  // prepare servos
  if (attachServos()) {
    writeServo1Command(currentAngle1);
    writeServo2Command(currentAngle2);
    lastWrittenAngle1 = currentAngle1;
    lastWrittenAngle2 = currentAngle2;
    lastServoCommandMs = millis();
    Serial.printf("Servo ready on GPIO %d/%d\n", servo1Pin, servo2Pin);
  } else {
    Serial.println("Servo attach failed (check wiring/power/pins)");
  }

  // pump init
  pinMode(pumpPin, OUTPUT);
  setPumpRunning(false);

  // onboard status LED init
  pinMode(statusLedPin, OUTPUT);
  digitalWrite(statusLedPin, LOW);

  // battery pin init
  pinMode(batteryPin, INPUT);
  analogReadResolution(12);

  delay(500);
}

void loop() {
  static unsigned long lastSweepMs = 0;
  static unsigned long lastServoSlewMs = 0;
  static unsigned long lastSensorLogMs = 0;
  const unsigned long sweepIntervalMs = 50;
  bool sensorUpdated = readSensors();
  bool rtcUpdated = readRTC();

  if (i2cPeriodicScanEnabled) {
    scanI2CBusPeriodic();
  }

  if (autoLevelEnabled && imuAnglesInitialized && !autoLevelReferenceCaptured) {
    if (autoLevelCaptureStartMs == 0) {
      autoLevelCaptureStartMs = millis();
    } else if ((millis() - autoLevelCaptureStartMs) >= autoLevelCaptureDelayMs) {
      autoLevelRollRefDeg = rollDeg;
      autoLevelPitchRefDeg = pitchDeg;
      autoLevelReferenceCaptured = true;
      Serial.printf("Auto-level reference set R=%.1f P=%.1f\n", autoLevelRollRefDeg, autoLevelPitchRefDeg);
    }
  }

  if (sensorSerialLogEnabled && (millis() - lastSensorLogMs) >= 1000) {
    lastSensorLogMs = millis();
    if (mpuReady) {
      Serial.printf("MPU a[g]=%.2f,%.2f,%.2f g[dps]=%.1f,%.1f,%.1f T=%.1fC\n",
                    accelX, accelY, accelZ, gyroX, gyroY, gyroZ, mpuTempC);
    }
    if (ahtReady) {
      Serial.printf("AHT20 T=%.1fC H=%.1f%%\n", ahtTempC, ahtHum);
    }
    if (bmpReady) {
      Serial.printf("BMP280 T=%.1fC P=%.1fhPa\n", bmpTempC, bmpPressureHPa);
    }
    if (rtcReady) {
      Serial.printf("RTC %04u-%02u-%02u %02u:%02u:%02u\n", rtcYear, rtcMonth, rtcDay, rtcHour, rtcMinute, rtcSecond);
    }
  }

  // Keep onboard LED steady ON while phone/BLE is connected
  if (bluetoothConnected) {
    if (!statusLedState) {
      statusLedState = true;
      digitalWrite(statusLedPin, HIGH);
    }
  } else if (statusLedState) {
    statusLedState = false;
    digitalWrite(statusLedPin, LOW);
  }

  eyeOpen = true;

  // Legacy directional control fallback (for single-char command sources)
  int targetCommand1 = currentAngle1;
  int targetCommand2 = currentAngle2;

  if (joystickActive) {
    if ((millis() - joystickLastUpdateMs) <= joystickActiveTimeoutMs) {
      targetCommand1 = joystickTargetAngle1;
      targetCommand2 = joystickTargetAngle2;
    } else {
      joystickActive = false;
    }
  }

  if (!joystickActive && leftPressed) {
    targetCommand1 = servoNeutralCommand + manualDriveOffset;
  } else if (!joystickActive && rightPressed) {
    targetCommand1 = servoNeutralCommand - manualDriveOffset;
  }

  if (!joystickActive && forwardPressed) {
    targetCommand2 = servoNeutralCommand + manualDriveOffset;
  } else if (!joystickActive && backPressed) {
    targetCommand2 = servoNeutralCommand - manualDriveOffset;
  }

  bool manualControlActive = joystickActive || leftPressed || rightPressed || forwardPressed || backPressed;

  if (autoLevelEnabled && autoLevelReferenceCaptured && !manualControlActive && !servo1Sweeping) {
    float rollError = rollDeg - autoLevelRollRefDeg;
    float pitchError = pitchDeg - autoLevelPitchRefDeg;

    if (fabsf(rollError) < autoLevelDeadbandDeg) rollError = 0.0f;
    if (fabsf(pitchError) < autoLevelDeadbandDeg) pitchError = 0.0f;

    int compS1 = constrain(static_cast<int>(roundf(rollError * autoLevelGainS1)), -autoLevelMaxCompDeg, autoLevelMaxCompDeg);
    int compS2 = constrain(static_cast<int>(roundf(pitchError * autoLevelGainS2)), -autoLevelMaxCompDeg, autoLevelMaxCompDeg);

    targetCommand1 = clampServoCommand(servoNeutralCommand + compS1);
    targetCommand2 = clampServoCommand(servoNeutralCommand + compS2);
  }

  if (servo1Sweeping) {
    targetCommand1 = currentAngle1;
  }

  // Servo1 sweep mode
  if (servo1Sweeping && (millis() - lastSweepMs) >= sweepIntervalMs) {
    lastSweepMs = millis();
    currentAngle1 += (sweepDirection * stepAngle);
    int sweepMax = servoNeutralCommand + sweepRangeOffset;
    int sweepMin = servoNeutralCommand - sweepRangeOffset;
    if (currentAngle1 >= sweepMax) {
      currentAngle1 = sweepMax;
      sweepDirection = -1;
    } else if (currentAngle1 <= sweepMin) {
      currentAngle1 = sweepMin;
      sweepDirection = 1;
    }
    targetCommand1 = currentAngle1;
  }

  int desiredAngle1 = currentAngle1;
  int desiredAngle2 = applyDeadband(clampServoCommand(targetCommand2), servo2Deadband);

  if (!servo1Sweeping) {
    desiredAngle1 = applyDeadband(clampServoCommand(targetCommand1), servo1Deadband);
  } else {
    desiredAngle1 = clampServoCommand(targetCommand1);
  }

  if ((millis() - lastServoSlewMs) >= servoSlewIntervalMs) {
    lastServoSlewMs = millis();
    bool activeControl = joystickActive || leftPressed || rightPressed || forwardPressed || backPressed || servo1Sweeping;
    int slewStep1 = activeControl ? servo1SlewStepActiveDeg : servoSlewStepIdleDeg;
    int slewStep2 = activeControl ? servo2SlewStepActiveDeg : servoSlewStepIdleDeg;
    currentAngle1 = moveToward(currentAngle1, desiredAngle1, slewStep1);
    currentAngle2 = moveToward(currentAngle2, desiredAngle2, slewStep2);
  }

  if (abs(currentAngle1 - lastWrittenAngle1) >= servo1CommandHysteresis) {
    ensureServosAttached();
    writeServo1Command(currentAngle1);
    lastWrittenAngle1 = currentAngle1;
    lastServoCommandMs = millis();
  }

  if (abs(currentAngle2 - lastWrittenAngle2) >= servo2CommandHysteresis) {
    ensureServosAttached();
    writeServo2Command(currentAngle2);
    lastWrittenAngle2 = currentAngle2;
    lastServoCommandMs = millis();
  }

  if (servoPwmDebugEnabled && (millis() - lastServoPwmDebugMs) >= servoPwmDebugIntervalMs) {
    lastServoPwmDebugMs = millis();
    Serial.printf("SERVO PWM s1=%ddeg/%dus s2=%ddeg/%dus attached=%d joy=%d\n",
                  currentAngle1,
                  lastServo1OutputUs,
                  currentAngle2,
                  lastServo2OutputUs,
                  servosAttached ? 1 : 0,
                  joystickActive ? 1 : 0);
  }

  bool noDirectionalInput = !leftPressed && !rightPressed && !forwardPressed && !backPressed;
  bool bothAtNeutral = abs(currentAngle1 - servoNeutralCommand) <= servoHomeDeadband &&
                       abs(currentAngle2 - servoNeutralCommand) <= servoHomeDeadband;
  if (servoAutoDetachEnabled && servosAttached && !servo1Sweeping && !joystickActive && noDirectionalInput && bothAtNeutral &&
      (millis() - lastServoCommandMs) >= servoAutoDetachMs) {
    myServo1.detach();
    myServo2.detach();
    servosAttached = false;
  }

  // Mark display dirty when sensor/RTC data changes
  if (sensorUpdated || rtcUpdated) {
    displayDirty = true;
  }

  if (mobileTimeUpdated) {
    displayDirty = true;
    mobileTimeUpdated = false;
  }

  if (bluetoothConnected && (millis() - lastTelemetryMs) >= telemetryIntervalMs) {
    lastTelemetryMs = millis();
    sendTelemetryPackets();
  }
  // ไม่ใช้ delay เพื่อให้ตอบสนองเร็ว
}

// ===== Display FreeRTOS Task (Core 0) =====
void displayTaskFunc(void *param) {
  display.begin(16000000);
  display.setSPISpeed(16000000);
  display.invertDisplay(false);
  display.setRotation(2);
  display.fillScreen(GC9A01A_BLACK);
  oledReady = true;
  Serial.printf("GC9A01 TFT ready on Core %d (CS=%d DC=%d RST=%d MOSI=%d SCLK=%d)\n",
                xPortGetCoreID(), tftCsPin, tftDcPin, tftRstPin, tftMosiPin, tftSclkPin);
  runOledStartupTest();
  displayDirty = true;

  int lastAngle1 = -1, lastAngle2 = -1;
  DisplayMode lastMode = MODE_BOOT;
  InfoPage lastInfoPage = INFO_PAGE_STATUS;
  bool lastPumpRunning = false;
  bool lastBleConnected = false;
  unsigned long cpuMeasureLastMs = millis();

  for (;;) {
    // Measure CPU load every ~1 second
    unsigned long now = millis();
    unsigned long elapsed = now - cpuMeasureLastMs;
    if (elapsed >= 1000 && cpuCalibIdlePerSec > 0) {
      float scale = 1000.0f / static_cast<float>(elapsed);
      float idle0 = static_cast<float>(cpuIdleCountCore0) * scale;
      float idle1 = static_cast<float>(cpuIdleCountCore1) * scale;
      cpuLoadCore0Pct = 100.0f - (idle0 / static_cast<float>(cpuCalibIdlePerSec)) * 100.0f;
      cpuLoadCore1Pct = 100.0f - (idle1 / static_cast<float>(cpuCalibIdlePerSec)) * 100.0f;
      if (cpuLoadCore0Pct < 0.0f) cpuLoadCore0Pct = 0.0f;
      if (cpuLoadCore1Pct < 0.0f) cpuLoadCore1Pct = 0.0f;
      if (cpuLoadCore0Pct > 100.0f) cpuLoadCore0Pct = 100.0f;
      if (cpuLoadCore1Pct > 100.0f) cpuLoadCore1Pct = 100.0f;
      cpuIdleCountCore0 = 0;
      cpuIdleCountCore1 = 0;
      cpuMeasureLastMs = now;
      displayDirty = true;
    }

    bool dirty = displayDirty;
    if (currentAngle1 != lastAngle1 || currentAngle2 != lastAngle2 ||
        currentMode != lastMode || currentInfoPage != lastInfoPage ||
        pumpRunning != lastPumpRunning || bluetoothConnected != lastBleConnected) {
      dirty = true;
    }

    if (dirty) {
      displayDirty = false;
      updateOLED();
      lastAngle1 = currentAngle1;
      lastAngle2 = currentAngle2;
      lastMode = currentMode;
      lastInfoPage = currentInfoPage;
      lastPumpRunning = pumpRunning;
      lastBleConnected = bluetoothConnected;
    }

    vTaskDelay(pdMS_TO_TICKS(80));  // ~12.5 FPS max, ลดภาระ CPU
  }
}

void updateOLED() {
  if (!displayEnabled || !oledReady) return;

  display.fillScreen(display.color565(6, 10, 24));

  for (int r = 112; r <= 118; r += 2) {
    display.drawCircle(120, 120, r, display.color565(18, 34, 68));
  }

  display.fillCircle(120, 120, 104, display.color565(8, 14, 32));
  display.setTextWrap(false);

  drawTopBarDateTime();

  drawInfoScreen();
}

void runOledStartupTest() {
  if (!oledReady || !oledStartupTestEnabled) return;

  display.fillScreen(display.color565(5, 12, 24));
  display.setTextColor(display.color565(160, 230, 255));
  display.setTextSize(2);
  display.setCursor(52, 34);
  display.print("GC9A01");
  display.drawRoundRect(26, 78, 188, 84, 14, display.color565(120, 210, 255));
  display.drawCircle(120, 120, 28, display.color565(120, 210, 255));
  display.setTextSize(1);
  display.setCursor(72, 150);
  display.setTextColor(display.color565(255, 216, 128));
  display.print("DISPLAY CHECK");
  delay(1200);

  display.fillScreen(display.color565(240, 248, 255));
  display.setTextColor(display.color565(8, 20, 46));
  display.setTextSize(2);
  display.setCursor(52, 110);
  display.print("WHITE OK");
  delay(900);

  display.fillScreen(display.color565(5, 12, 24));
  display.setTextColor(display.color565(170, 255, 210));
  display.setTextSize(2);
  display.setCursor(40, 110);
  display.print("STARTING...");
  delay(900);
}

void drawBootScreen() {
  if (pumpRunning) {
    drawRotatingFrame();
    drawAngryFace();
  } else {
    drawStaticFrame();
    drawHappyFace();
  }
}

void drawStaticFrame() {
  display.drawCircle(120, 120, 88, display.color565(70, 130, 240));
  display.drawCircle(120, 120, 74, display.color565(38, 80, 170));
}

void drawRotatingFrame() {
  const int segments = 20;
  const float phase = static_cast<float>(framePhase) / static_cast<float>(framePhaseMax);
  const float twoPi = 6.2831853f;

  for (int i = 0; i < segments; i++) {
    float t = static_cast<float>(i) / static_cast<float>(segments);
    float ang = (t + phase) * twoPi;
    int x = 120 + static_cast<int>(roundf(cosf(ang) * 88.0f));
    int y = 120 + static_cast<int>(roundf(sinf(ang) * 88.0f));
    uint16_t c = (i % 3 == 0) ? display.color565(255, 92, 120)
                              : display.color565(64, 176, 255);
    display.fillCircle(x, y, 3, c);
  }
}

void drawHappyFace() {
  int centerX = 120;
  int centerY = 128;
  
  // Draw left eye (larger, with blink animation)
  if (eyeOpen) {
    display.fillCircle(centerX - 25, centerY - 14, 9, display.color565(255, 255, 255));
    display.fillCircle(centerX - 25, centerY - 14, 5, display.color565(10, 18, 36));
    display.drawPixel(centerX - 22, centerY - 17, display.color565(255, 255, 255));
  } else {
    display.drawLine(centerX - 34, centerY - 14, centerX - 16, centerY - 14, display.color565(255, 255, 255));
  }
  
  // Draw right eye (larger, with blink animation)
  if (eyeOpen) {
    display.fillCircle(centerX + 25, centerY - 14, 9, display.color565(255, 255, 255));
    display.fillCircle(centerX + 25, centerY - 14, 5, display.color565(10, 18, 36));
    display.drawPixel(centerX + 22, centerY - 17, display.color565(255, 255, 255));
  } else {
    display.drawLine(centerX + 16, centerY - 14, centerX + 34, centerY - 14, display.color565(255, 255, 255));
  }

  display.drawLine(centerX - 16, centerY + 8, centerX - 8, centerY + 18, display.color565(90, 255, 180));
  display.drawLine(centerX - 8, centerY + 18, centerX + 8, centerY + 18, display.color565(90, 255, 180));
  display.drawLine(centerX + 8, centerY + 18, centerX + 16, centerY + 8, display.color565(90, 255, 180));
  display.drawLine(centerX - 14, centerY + 15, centerX + 14, centerY + 15, display.color565(90, 255, 180));
}

void drawAngryFace() {
  int centerX = 120;
  int centerY = 128;

  display.drawLine(centerX - 34, centerY - 22, centerX - 16, centerY - 16, display.color565(255, 110, 120));
  display.drawLine(centerX + 16, centerY - 16, centerX + 34, centerY - 22, display.color565(255, 110, 120));
  
  // Draw eyes (angry, with blink animation)
  if (eyeOpen) {
    display.fillCircle(centerX - 25, centerY - 12, 9, display.color565(255, 255, 255));
    display.fillCircle(centerX - 25, centerY - 12, 5, display.color565(10, 18, 36));
    display.drawPixel(centerX - 22, centerY - 10, display.color565(255, 255, 255));

    display.fillCircle(centerX + 25, centerY - 12, 9, display.color565(255, 255, 255));
    display.fillCircle(centerX + 25, centerY - 12, 5, display.color565(10, 18, 36));
    display.drawPixel(centerX + 28, centerY - 10, display.color565(255, 255, 255));
  } else {
    display.drawLine(centerX - 34, centerY - 12, centerX - 16, centerY - 12, display.color565(255, 255, 255));
    display.drawLine(centerX + 16, centerY - 12, centerX + 34, centerY - 12, display.color565(255, 255, 255));
  }

  display.drawLine(centerX - 16, centerY + 20, centerX - 8, centerY + 8, display.color565(255, 90, 110));
  display.drawLine(centerX - 8, centerY + 8, centerX + 8, centerY + 8, display.color565(255, 90, 110));
  display.drawLine(centerX + 8, centerY + 8, centerX + 16, centerY + 20, display.color565(255, 90, 110));
}

void drawInfoScreen() {
  const int contentY = 52;
  const int panelX = 34;
  const int panelW = 172;
  const int panelH = 136;
  display.setTextSize(1);

  if (currentInfoPage == INFO_PAGE_STATUS) {
    int s1 = commandToPercent(currentAngle1);
    int s2 = commandToPercent(currentAngle2);

    display.fillRoundRect(panelX, contentY, panelW, panelH, 12, display.color565(20, 34, 70));
    display.drawRoundRect(panelX, contentY, panelW, panelH, 12, display.color565(88, 160, 255));
    display.setTextColor(display.color565(188, 228, 255));
    display.setCursor(48, contentY + 10);
    display.printf("S1 %4d%%", s1);
    display.setCursor(122, contentY + 10);
    display.printf("S2 %4d%%", s2);

    int barW1 = map(abs(s1), 0, 100, 0, 56);
    int barW2 = map(abs(s2), 0, 100, 0, 56);
    uint16_t c1 = s1 >= 0 ? display.color565(96, 240, 178) : display.color565(255, 140, 120);
    uint16_t c2 = s2 >= 0 ? display.color565(96, 240, 178) : display.color565(255, 140, 120);
    display.drawRect(48, contentY + 30, 56, 10, display.color565(80, 120, 180));
    display.drawRect(122, contentY + 30, 56, 10, display.color565(80, 120, 180));
    display.fillRect(48, contentY + 30, barW1, 10, c1);
    display.fillRect(122, contentY + 30, barW2, 10, c2);

    display.fillRoundRect(42, contentY + 54, 136, 62, 10, display.color565(24, 44, 54));
    display.setTextColor(display.color565(170, 250, 220));
    display.setCursor(52, contentY + 68);
    display.printf("BAT %dmV", batteryVoltage);
    display.setCursor(122, contentY + 68);
    display.print(bluetoothConnected ? "BLE ON" : "BLE OFF");
    display.setCursor(52, contentY + 88);
    display.printf("PUMP %s  AB %s", pumpRunning ? "ON" : "OFF", autoLevelEnabled ? "ON" : "OFF");

    display.setTextColor(display.color565(255, 214, 126));
    display.setCursor(72, 198);
    display.print("STATUS 1/4");
  } else if (currentInfoPage == INFO_PAGE_SENSORS) {
    display.fillRoundRect(panelX, contentY, panelW, panelH, 12, display.color565(30, 30, 56));
    display.drawRoundRect(panelX, contentY, panelW, panelH, 12, display.color565(130, 130, 240));

    display.setTextColor(display.color565(180, 220, 255));
    display.setCursor(82, contentY + 10);
    display.print("SENSORS");

    display.setTextColor(display.color565(255, 255, 255));
    display.setCursor(44, contentY + 34);
    if (ahtReady) {
      display.printf("AHT  T:%4.1fC  H:%2.0f%%", ahtTempC, ahtHum);
    } else {
      display.print("AHT  NOT FOUND");
    }

    display.setCursor(44, contentY + 56);
    if (bmpReady) {
      display.printf("BMP  P:%4.0fhPa", bmpPressureHPa);
    } else {
      display.print("BMP  NOT FOUND");
    }

    display.setCursor(44, contentY + 78);
    if (mpuReady) {
      display.printf("MPU  T:%4.1fC", mpuTempC);
    } else {
      display.print("MPU  NOT FOUND");
    }

    display.setCursor(44, contentY + 100);
    display.printf("RTC  %s", rtcReady ? "READY" : "NOT READY");

    display.setTextColor(display.color565(255, 214, 126));
    display.setCursor(72, 198);
    display.print("SENSOR 2/4");
  } else if (currentInfoPage == INFO_PAGE_IMU) {
    display.fillRoundRect(panelX, contentY, panelW, panelH, 12, display.color565(42, 24, 38));
    display.drawRoundRect(panelX, contentY, panelW, panelH, 12, display.color565(255, 120, 180));

    display.setTextColor(display.color565(255, 210, 238));
    display.setCursor(102, contentY + 10);
    display.print("IMU");

    int rollBar = map(constrain(static_cast<int>(roundf(rollDeg)), -90, 90), -90, 90, 0, 120);
    int pitchBar = map(constrain(static_cast<int>(roundf(pitchDeg)), -90, 90), -90, 90, 0, 120);

    display.drawRect(60, contentY + 30, 120, 10, display.color565(130, 80, 130));
    display.fillRect(60, contentY + 30, rollBar, 10, display.color565(255, 130, 170));
    display.drawRect(60, contentY + 54, 120, 10, display.color565(130, 80, 130));
    display.fillRect(60, contentY + 54, pitchBar, 10, display.color565(255, 170, 110));

    display.setTextColor(display.color565(255, 238, 250));
    display.setCursor(44, contentY + 76);
    display.printf("ROLL  %6.1f", rollDeg);
    display.setCursor(44, contentY + 96);
    display.printf("PITCH %5.1f", pitchDeg);
    display.setCursor(44, contentY + 116);
    display.printf("GYRO Y:%5.1f Z:%5.1f", gyroY, gyroZ);

    display.setTextColor(display.color565(255, 214, 126));
    display.setCursor(84, 198);
    display.print("IMU 3/4");
  } else if (currentInfoPage == INFO_PAGE_CPU) {
    display.fillRoundRect(panelX, contentY, panelW, panelH, 12, display.color565(18, 36, 28));
    display.drawRoundRect(panelX, contentY, panelW, panelH, 12, display.color565(80, 255, 160));

    display.setTextColor(display.color565(160, 255, 200));
    display.setCursor(72, contentY + 10);
    display.print("CPU MONITOR");

    // Core 0 (Display + WiFi)
    display.setTextColor(display.color565(255, 255, 255));
    display.setCursor(44, contentY + 32);
    display.printf("Core 0  %5.1f%%", cpuLoadCore0Pct);
    int bar0W = map(constrain(static_cast<int>(roundf(cpuLoadCore0Pct)), 0, 100), 0, 100, 0, 120);
    display.drawRect(60, contentY + 44, 120, 10, display.color565(50, 100, 70));
    uint16_t barColor0 = cpuLoadCore0Pct > 80.0f ? display.color565(255, 80, 80) :
                         cpuLoadCore0Pct > 50.0f ? display.color565(255, 200, 60) :
                                                   display.color565(80, 255, 140);
    display.fillRect(60, contentY + 44, bar0W, 10, barColor0);

    // Core 1 (Main loop + BLE + Servo)
    display.setCursor(44, contentY + 62);
    display.printf("Core 1  %5.1f%%", cpuLoadCore1Pct);
    int bar1W = map(constrain(static_cast<int>(roundf(cpuLoadCore1Pct)), 0, 100), 0, 100, 0, 120);
    display.drawRect(60, contentY + 74, 120, 10, display.color565(50, 100, 70));
    uint16_t barColor1 = cpuLoadCore1Pct > 80.0f ? display.color565(255, 80, 80) :
                         cpuLoadCore1Pct > 50.0f ? display.color565(255, 200, 60) :
                                                   display.color565(80, 255, 140);
    display.fillRect(60, contentY + 74, bar1W, 10, barColor1);

    // Free heap
    display.setTextColor(display.color565(200, 240, 220));
    display.setCursor(44, contentY + 94);
    display.printf("Heap  %6lu B", (unsigned long)ESP.getFreeHeap());
    display.setCursor(44, contentY + 110);
    display.printf("Stack D:%4lu L:%4lu",
                   (unsigned long)uxTaskGetStackHighWaterMark(displayTaskHandle),
                   (unsigned long)uxTaskGetStackHighWaterMark(nullptr));

    display.setTextColor(display.color565(255, 214, 126));
    display.setCursor(82, 198);
    display.print("CPU 4/4");
  }
}

void drawTopBarDateTime() {
  bool useMobileTime = bluetoothConnected && mobileTimeValid && ((millis() - mobileTimeLastSyncMs) <= 3000);
  display.fillRoundRect(42, 24, 156, 20, 8, display.color565(25, 48, 92));
  display.drawRoundRect(42, 24, 156, 20, 8, display.color565(78, 170, 255));
  display.setTextColor(display.color565(220, 245, 255));
  display.setTextSize(1);

  if (useMobileTime || rtcReady) {
    char topLine[24];
    uint16_t year = useMobileTime ? mobileYear : rtcYear;
    uint8_t month = useMobileTime ? mobileMonth : rtcMonth;
    uint8_t day = useMobileTime ? mobileDay : rtcDay;
    uint8_t hour = useMobileTime ? mobileHour : rtcHour;
    uint8_t minute = useMobileTime ? mobileMinute : rtcMinute;
    uint8_t second = useMobileTime ? mobileSecond : rtcSecond;

    snprintf(topLine, sizeof(topLine), "%02u/%02u/%04u %02u:%02u:%02u", day, month, year, hour, minute, second);

    int lineWidth = static_cast<int>(strlen(topLine)) * 6;
    int lineX = 120 - (lineWidth / 2);
    if (lineX < 48) lineX = 48;

    display.setCursor(lineX, 30);
    display.print(topLine);
  } else {
    const char *noRtc = "--/--/---- --:--:--";
    int lineWidth = static_cast<int>(strlen(noRtc)) * 6;
    int lineX = 120 - (lineWidth / 2);
    if (lineX < 48) lineX = 48;
    display.setCursor(lineX, 30);
    display.print(noRtc);
  }

  display.setTextColor(display.color565(235, 244, 255));
}

void processCommand(char cmd) {
  if (cmd == '\r' || cmd == '\n' || cmd == ' ') return;

  Serial.printf("Command RX: %c\n", cmd);
  
  char normalized = toupper(static_cast<unsigned char>(cmd));

  if (normalized == 'L') {
    joystickActive = false;
    leftPressed = true;
    rightPressed = false;
    forwardPressed = false;
    backPressed = false;
  } else if (normalized == 'R') {
    joystickActive = false;
    leftPressed = false;
    rightPressed = true;
    forwardPressed = false;
    backPressed = false;
  } else if (normalized == 'F') {
    joystickActive = false;
    leftPressed = false;
    rightPressed = false;
    forwardPressed = true;
    backPressed = false;
  } else if (normalized == 'B') {
    joystickActive = false;
    leftPressed = false;
    rightPressed = false;
    forwardPressed = false;
    backPressed = true;
  } else if (normalized == '0' || normalized == 'S') {
    joystickActive = false;
    leftPressed = false;
    rightPressed = false;
    forwardPressed = false;
    backPressed = false;
  } else if (normalized == 'T') {
    // Reset to center (square button)
    servo1Sweeping = false;
    joystickActive = false;
    currentAngle1 = servoNeutralCommand;
    currentAngle2 = servoNeutralCommand;
    ensureServosAttached();
    writeServo1Command(currentAngle1);
    writeServo2Command(currentAngle2);
    lastWrittenAngle1 = currentAngle1;
    lastWrittenAngle2 = currentAngle2;
    lastServoCommandMs = millis();
    Serial.println("Reset to center");
  } else if (normalized == 'U') {
    // Toggle servo1 sweep (triangle button)
    servo1Sweeping = !servo1Sweeping;
    if (servo1Sweeping) {
      sweepDirection = 1;
      Serial.println("Servo1 sweep ON");
    } else {
      Serial.println("Servo1 sweep OFF");
    }
  } else if (normalized == 'M') {
    // Pump ON (circle button)
    setPumpRunning(true);
    Serial.println("Pump ON");
  } else if (normalized == 'N') {
    // Pump OFF (X button)
    setPumpRunning(false);
    Serial.println("Pump OFF");
  } else if (normalized == 'P') {
    currentMode = MODE_INFO;
    currentInfoPage = static_cast<InfoPage>((currentInfoPage + 1) % INFO_PAGE_MAX);
    if (currentInfoPage == INFO_PAGE_STATUS) {
      Serial.println("Info page STATUS");
    } else if (currentInfoPage == INFO_PAGE_SENSORS) {
      Serial.println("Info page SENSOR");
    } else if (currentInfoPage == INFO_PAGE_IMU) {
      Serial.println("Info page IMU");
    } else {
      Serial.println("Info page CPU");
    }
  } else if (normalized == 'Q') {
    autoLevelEnabled = !autoLevelEnabled;
    autoLevelReferenceCaptured = false;
    autoLevelCaptureStartMs = 0;
    Serial.printf("Auto Balance %s\n", autoLevelEnabled ? "ON" : "OFF");
  } else if (normalized == 'C') {
    // BLE connected
    bluetoothConnected = true;
    Serial.println("BLE Connected");
  } else if (normalized == 'D') {
    // BLE disconnected
    bluetoothConnected = false;
    mobileTimeValid = false;
    Serial.println("BLE Disconnected");
  }
}

void setPumpRunning(bool enabled) {
  pumpRunning = enabled;
  bool outputOn = pumpRelayActiveLow ? !enabled : enabled;
  digitalWrite(pumpPin, outputOn ? HIGH : LOW);
}

void processPacket(const std::string &packet) {
  if (packet.empty()) return;

  Serial.printf("Packet RX: %s\n", packet.c_str());

  if (packet[0] == 'J' || packet[0] == 'j') {
    size_t firstComma = packet.find(',');
    size_t secondComma = packet.find(',', firstComma == std::string::npos ? 0 : firstComma + 1);
    if (firstComma == std::string::npos || secondComma == std::string::npos) return;

    std::string xStr = packet.substr(firstComma + 1, secondComma - firstComma - 1);
    std::string yStr = packet.substr(secondComma + 1);
    int xNorm = atoi(xStr.c_str());
    int yNorm = atoi(yStr.c_str());

    xNorm = constrain(xNorm, -100, 100);
    yNorm = constrain(yNorm, -100, 100);

    updateJoystickInputStats(xNorm, yNorm);

    leftPressed = false;
    rightPressed = false;
    forwardPressed = false;
    backPressed = false;

    setServoAnglesFromNormalized(xNorm, yNorm);
    return;
  }

  if (packet[0] == 'Z' || packet[0] == 'z') {
    int year = 0;
    int month = 0;
    int day = 0;
    int hour = 0;
    int minute = 0;
    int second = 0;
    int parsed = sscanf(packet.c_str(), "%*[^,],%d,%d,%d,%d,%d,%d",
                        &year, &month, &day, &hour, &minute, &second);

    if (parsed == 6 &&
        year >= 2000 && year <= 2099 &&
        month >= 1 && month <= 12 &&
        day >= 1 && day <= 31 &&
        hour >= 0 && hour <= 23 &&
        minute >= 0 && minute <= 59 &&
        second >= 0 && second <= 59) {
      setMobileDateTime(year, month, day, hour, minute, second);
    }
    return;
  }

  if (packet.size() == 1) {
    processCommand(packet[0]);
  }
}

void setMobileDateTime(int year, int month, int day, int hour, int minute, int second) {
  mobileYear = static_cast<uint16_t>(year);
  mobileMonth = static_cast<uint8_t>(month);
  mobileDay = static_cast<uint8_t>(day);
  mobileHour = static_cast<uint8_t>(hour);
  mobileMinute = static_cast<uint8_t>(minute);
  mobileSecond = static_cast<uint8_t>(second);
  mobileTimeLastSyncMs = millis();
  mobileTimeValid = true;
  mobileTimeUpdated = true;
  bluetoothConnected = true;

  RtcDateTime mobileNow(mobileYear, mobileMonth, mobileDay, mobileHour, mobileMinute, mobileSecond);
  rtc.SetDateTime(mobileNow);

  rtcYear = mobileYear;
  rtcMonth = mobileMonth;
  rtcDay = mobileDay;
  rtcHour = mobileHour;
  rtcMinute = mobileMinute;
  rtcSecond = mobileSecond;
  rtcReady = true;
}

void setServoAnglesFromNormalized(int xNorm, int yNorm) {
  if (abs(xNorm) <= joystickNormDeadzone) xNorm = 0;
  if (abs(yNorm) <= joystickNormDeadzone) yNorm = 0;

  joystickFilteredXNorm = xNorm;
  joystickFilteredYNorm = yNorm;

  int absX = abs(xNorm);
  int absY = abs(yNorm);
  bool updateServo1 = absX >= absY;
  bool updateServo2 = absY >= absX;

  int targetAngle1 = map(xNorm, -100, 100, minAngle, maxAngle);
  int targetAngle2 = map(yNorm, -100, 100, maxAngle, minAngle);

  targetAngle1 = quantizeCommand(targetAngle1, joystickCommandStepDeg);
  targetAngle2 = quantizeCommand(targetAngle2, joystickCommandStepDeg);

  targetAngle1 = applyDeadband(clampServoCommand(targetAngle1), servo1Deadband);
  targetAngle2 = applyDeadband(clampServoCommand(targetAngle2), servo2Deadband);

  if (!servo1Sweeping && updateServo1) {
    joystickTargetAngle1 = targetAngle1;
  } else {
    joystickTargetAngle1 = currentAngle1;
  }

  if (updateServo2) {
    joystickTargetAngle2 = targetAngle2;
  } else {
    joystickTargetAngle2 = currentAngle2;
  }

  currentAngle1 = joystickTargetAngle1;
  currentAngle2 = joystickTargetAngle2;
  ensureServosAttached();
  writeServo1Command(currentAngle1);
  writeServo2Command(currentAngle2);
  lastWrittenAngle1 = currentAngle1;
  lastWrittenAngle2 = currentAngle2;
  lastServoCommandMs = millis();

  joystickLastUpdateMs = millis();
  joystickActive = true;
}

void updateJoystickInputStats(int rawXNorm, int rawYNorm) {
  joystickRawXNorm = rawXNorm;
  joystickRawYNorm = rawYNorm;

  if (!joystickInputStatsEnabled) return;

  unsigned long nowMs = millis();
  if (joystickStatsWindowStartMs == 0) {
    joystickStatsWindowStartMs = nowMs;
  }

  joystickStatsPacketCount++;

  if (joystickStatsHasPrevSample) {
    int deltaX = abs(rawXNorm - joystickStatsPrevXNorm);
    int deltaY = abs(rawYNorm - joystickStatsPrevYNorm);
    joystickStatsAbsDeltaXSum += static_cast<unsigned long>(deltaX);
    joystickStatsAbsDeltaYSum += static_cast<unsigned long>(deltaY);
    joystickStatsDeltaXMax = max(joystickStatsDeltaXMax, deltaX);
    joystickStatsDeltaYMax = max(joystickStatsDeltaYMax, deltaY);

    unsigned long dtMs = nowMs - joystickStatsLastPacketMs;
    joystickStatsDtSumMs += dtMs;
    if (joystickStatsDtMinMs == 0 || dtMs < joystickStatsDtMinMs) {
      joystickStatsDtMinMs = dtMs;
    }
    if (dtMs > joystickStatsDtMaxMs) {
      joystickStatsDtMaxMs = dtMs;
    }
  }

  joystickStatsPrevXNorm = rawXNorm;
  joystickStatsPrevYNorm = rawYNorm;
  joystickStatsLastPacketMs = nowMs;
  joystickStatsHasPrevSample = true;

  if ((nowMs - joystickStatsWindowStartMs) >= joystickInputStatsIntervalMs) {
    reportJoystickInputStats();
  }
}

void reportJoystickInputStats() {
  if (!joystickInputStatsEnabled) return;

  unsigned int deltaSamples = joystickStatsPacketCount > 0 ? (joystickStatsPacketCount - 1) : 0;
  float avgAbsDeltaX = deltaSamples > 0 ?
    static_cast<float>(joystickStatsAbsDeltaXSum) / static_cast<float>(deltaSamples) : 0.0f;
  float avgAbsDeltaY = deltaSamples > 0 ?
    static_cast<float>(joystickStatsAbsDeltaYSum) / static_cast<float>(deltaSamples) : 0.0f;
  float avgDtMs = deltaSamples > 0 ?
    static_cast<float>(joystickStatsDtSumMs) / static_cast<float>(deltaSamples) : 0.0f;

  Serial.printf("JOY in raw=(%d,%d) filt=(%d,%d) pkt=%u dAvg=(%.1f,%.1f) dMax=(%d,%d) dtAvg=%.1f dtMin=%lu dtMax=%lu\\n",
                joystickRawXNorm,
                joystickRawYNorm,
                joystickFilteredXNorm,
                joystickFilteredYNorm,
                joystickStatsPacketCount,
                avgAbsDeltaX,
                avgAbsDeltaY,
                joystickStatsDeltaXMax,
                joystickStatsDeltaYMax,
                avgDtMs,
                joystickStatsDtMinMs,
                joystickStatsDtMaxMs);

  joystickStatsWindowStartMs = millis();
  joystickStatsPacketCount = 0;
  joystickStatsDtSumMs = 0;
  joystickStatsDtMinMs = 0;
  joystickStatsDtMaxMs = 0;
  joystickStatsAbsDeltaXSum = 0;
  joystickStatsAbsDeltaYSum = 0;
  joystickStatsDeltaXMax = 0;
  joystickStatsDeltaYMax = 0;
}

void ensureServosAttached() {
  if (servosAttached) return;
  if (!attachServos()) {
    Serial.println("Servo re-attach failed");
  }
}

bool attachServos() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  myServo1.setPeriodHertz(50);
  myServo2.setPeriodHertz(50);

  int channel1 = myServo1.attach(servo1Pin, servoPulseMinUs, servoPulseMaxUs);
  int channel2 = myServo2.attach(servo2Pin, servoPulseMinUs, servoPulseMaxUs);
  servosAttached = (channel1 >= 0) && (channel2 >= 0);
  return servosAttached;
}

int moveToward(int currentValue, int targetValue, int maxStep) {
  if (currentValue < targetValue) {
    return min(currentValue + maxStep, targetValue);
  }
  if (currentValue > targetValue) {
    return max(currentValue - maxStep, targetValue);
  }
  return currentValue;
}

int clampServoCommand(int command) {
  return constrain(command, servoMinCommand, servoMaxCommand);
}

int toServo1OutputCommand(int logicalCommand) {
  int clamped = clampServoCommand(logicalCommand);
  return servoMaxCommand - (clamped - servoMinCommand);
}

void writeServo1Command(int logicalCommand) {
  int reversed = toServo1OutputCommand(logicalCommand);
  int limited = constrain(reversed, servo1SoftMinCommand, servo1SoftMaxCommand);
  int outputUs = map(limited, servoMinCommand, servoMaxCommand, servo1OutputMinUs, servo1OutputMaxUs);
  if (abs(logicalCommand - servoNeutralCommand) <= servoHomeDeadband) {
    outputUs = 1500 + servo1CenterTrimUs;
  } else {
    outputUs += servo1CenterTrimUs;
  }
  outputUs = constrain(outputUs, servo1OutputMinUs, servo1OutputMaxUs);
  lastServo1OutputUs = outputUs;
  myServo1.writeMicroseconds(outputUs);
}

void writeServo2Command(int logicalCommand) {
  int limited = constrain(logicalCommand, servo2SoftMinCommand, servo2SoftMaxCommand);
  int outputUs = map(limited, servoMinCommand, servoMaxCommand, servo2OutputMinUs, servo2OutputMaxUs);
  if (abs(logicalCommand - servoNeutralCommand) <= servoHomeDeadband) {
    outputUs = 1500 + servo2CenterTrimUs;
  } else {
    outputUs += servo2CenterTrimUs;
  }
  outputUs = constrain(outputUs, servo2OutputMinUs, servo2OutputMaxUs);
  lastServo2OutputUs = outputUs;
  myServo2.writeMicroseconds(outputUs);
}

int applyDeadband(int command, int deadbandMargin) {
  if (abs(command - servoNeutralCommand) <= deadbandMargin) {
    return servoNeutralCommand;
  }
  return command;
}

int quantizeCommand(int command, int stepSize) {
  if (stepSize <= 1) return command;
  int clamped = clampServoCommand(command);
  int q = (clamped + (stepSize / 2)) / stepSize;
  return clampServoCommand(q * stepSize);
}

int commandToPercent(int command) {
  int clamped = clampServoCommand(command);
  return map(clamped, servoMinCommand, servoMaxCommand, -100, 100);
}

void scanI2CBus() {
  Serial.println("I2C scan start...");
  uint8_t foundCount = 0;

  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();

    if (error == 0) {
      foundCount++;
      Serial.printf("I2C device found at 0x%02X\n", address);
    } else if (error == 4) {
      Serial.printf("I2C unknown error at 0x%02X\n", address);
    }

    delay(1);
  }

  if (foundCount == 0) {
    Serial.println("I2C scan complete: no devices found");
  } else {
    Serial.printf("I2C scan complete: %u device(s)\n", foundCount);
  }
}

void scanI2CBusPeriodic() {
  if (!i2cPeriodicScanEnabled) return;
  if ((millis() - lastI2CPeriodicScanMs) < i2cPeriodicScanIntervalMs) return;
  lastI2CPeriodicScanMs = millis();

  bool first = true;
  char line[128];
  size_t used = 0;
  used += snprintf(line + used, sizeof(line) - used, "I2C[2s]:");

  uint8_t foundCount = 0;
  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();
    if (error == 0) {
      foundCount++;
      used += snprintf(line + used, sizeof(line) - used, "%s0x%02X", first ? " " : ",", address);
      first = false;
      if (used >= sizeof(line) - 6) {
        Serial.println(line);
        used = snprintf(line, sizeof(line), "I2C[2s]-cont:");
      }
    }
  }

  if (foundCount == 0) {
    Serial.println("I2C[2s]: none");
  } else {
    Serial.println(line);
  }
}

void initSensors() {
  mpuFallbackMode = false;
  mpuWhoAmI = 0;

  if (mpu.begin(0x68, &Wire)) {
    mpuReady = true;
    mpuI2CAddress = 0x68;
  } else if (mpu.begin(0x69, &Wire)) {
    mpuReady = true;
    mpuI2CAddress = 0x69;
  }

  if (!mpuReady) {
    uint8_t who = 0;
    if (readMpuWhoAmI(0x68, who)) {
      mpuI2CAddress = 0x68;
      mpuWhoAmI = who;
      mpuReady = true;
      mpuFallbackMode = true;
    } else if (readMpuWhoAmI(0x69, who)) {
      mpuI2CAddress = 0x69;
      mpuWhoAmI = who;
      mpuReady = true;
      mpuFallbackMode = true;
    }

    if (mpuFallbackMode) {
      Wire.beginTransmission(mpuI2CAddress);
      Wire.write(0x6B);
      Wire.write(0x00);
      Wire.endTransmission();

      Wire.beginTransmission(mpuI2CAddress);
      Wire.write(0x1B);
      Wire.write(0x08);
      Wire.endTransmission();

      Wire.beginTransmission(mpuI2CAddress);
      Wire.write(0x1C);
      Wire.write(0x10);
      Wire.endTransmission();

      Wire.beginTransmission(mpuI2CAddress);
      Wire.write(0x1A);
      Wire.write(0x04);
      Wire.endTransmission();
    }
  }

  if (mpuReady) {
    if (!mpuFallbackMode) {
      mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
      mpu.setGyroRange(MPU6050_RANGE_500_DEG);
      mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
      Serial.printf("MPU6050 ready at 0x%02X\n", mpuI2CAddress);
    } else {
      Serial.printf("MPU fallback ready at 0x%02X WHO_AM_I=0x%02X\n", mpuI2CAddress, mpuWhoAmI);
    }
  } else {
    Serial.println("MPU6050 not found (0x68/0x69)");
  }

  ahtReady = aht.begin(&Wire);
  if (ahtReady) {
    sensors_event_t humidity;
    sensors_event_t temp;
    bool probeOk = aht.getEvent(&humidity, &temp) &&
                   isfinite(temp.temperature) &&
                   isfinite(humidity.relative_humidity);
    if (probeOk) {
      ahtTempC = temp.temperature;
      ahtHum = humidity.relative_humidity;
      ahtReadFailCount = 0;
      Serial.println("AHT20 ready");
    } else {
      ahtReady = false;
      Serial.println("AHT20 probe failed, disabled");
    }
  } else {
    Serial.println("AHT20 not found");
  }

  if (bmp.begin(0x76, BMP280_CHIPID)) {
    bmpReady = true;
    bmpI2CAddress = 0x76;
  } else if (bmp.begin(0x77, BMP280_CHIPID)) {
    bmpReady = true;
    bmpI2CAddress = 0x77;
  }

  if (bmpReady) {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_125);
    float probeTemp = bmp.readTemperature();
    float probePressure = bmp.readPressure() / 100.0f;
    bool probeOk = isfinite(probeTemp) && isfinite(probePressure) &&
                   probePressure >= 300.0f && probePressure <= 1200.0f;
    if (probeOk) {
      bmpTempC = probeTemp;
      bmpPressureHPa = probePressure;
      bmpReadFailCount = 0;
      Serial.printf("BMP280 ready at 0x%02X\n", bmpI2CAddress);
    } else {
      bmpReady = false;
      Serial.println("BMP280 probe failed, disabled");
    }
  } else {
    Serial.println("BMP280 not found (0x76/0x77)");
  }
}

bool readSensors() {
  static unsigned long lastReadMs = 0;
  const unsigned long readIntervalMs = 180;

  if (reduceI2CWhileServoActive) {
    bool servoBusy = servo1Sweeping || joystickActive || leftPressed || rightPressed || forwardPressed || backPressed;
    if (servoBusy) return false;
  }

  if ((millis() - lastReadMs) < readIntervalMs) return false;
  lastReadMs = millis();

  bool updated = false;
  if (mpuReady) {
    if (!mpuFallbackMode) {
      sensors_event_t accel;
      sensors_event_t gyro;
      sensors_event_t temp;
      mpu.getEvent(&accel, &gyro, &temp);
      accelX = accel.acceleration.x / 9.80665f;
      accelY = accel.acceleration.y / 9.80665f;
      accelZ = accel.acceleration.z / 9.80665f;
      gyroX = gyro.gyro.x * 57.29578f;
      gyroY = gyro.gyro.y * 57.29578f;
      gyroZ = gyro.gyro.z * 57.29578f;
      mpuTempC = temp.temperature;
      updated = true;
    } else {
      updated = readMpuRawSample() || updated;
    }

    if (updated) {
      unsigned long nowMs = millis();
      float dt = 0.0f;
      if (lastImuUpdateMs > 0) {
        dt = static_cast<float>(nowMs - lastImuUpdateMs) / 1000.0f;
      }
      lastImuUpdateMs = nowMs;

      float rollAcc = atan2f(accelY, accelZ) * 57.29578f;
      float pitchAcc = atan2f(-accelX, sqrtf((accelY * accelY) + (accelZ * accelZ))) * 57.29578f;

      if (!imuAnglesInitialized || dt <= 0.0f || dt > 0.5f) {
        rollDeg = rollAcc;
        pitchDeg = pitchAcc;
        imuAnglesInitialized = true;
      } else {
        const float alpha = 0.96f;
        float rollGyro = rollDeg + (gyroX * dt);
        float pitchGyro = pitchDeg + (gyroY * dt);
        rollDeg = (alpha * rollGyro) + ((1.0f - alpha) * rollAcc);
        pitchDeg = (alpha * pitchGyro) + ((1.0f - alpha) * pitchAcc);
      }
    }
  }

  if (ahtReady) {
    sensors_event_t humidity;
    sensors_event_t temp;
    bool ok = aht.getEvent(&humidity, &temp) &&
              isfinite(temp.temperature) &&
              isfinite(humidity.relative_humidity) &&
              humidity.relative_humidity >= 0.0f &&
              humidity.relative_humidity <= 100.0f;
    if (ok) {
      ahtTempC = temp.temperature;
      ahtHum = humidity.relative_humidity;
      ahtReadFailCount = 0;
      updated = true;
    } else {
      if (ahtReadFailCount < 255) ahtReadFailCount++;
      if (ahtReadFailCount >= i2cSensorFailThreshold) {
        ahtReady = false;
        Serial.println("AHT20 disabled after repeated I2C read failures");
      }
    }
  }

  if (bmpReady) {
    float nextBmpTemp = bmp.readTemperature();
    float nextBmpPressure = bmp.readPressure() / 100.0f;
    bool ok = isfinite(nextBmpTemp) &&
              isfinite(nextBmpPressure) &&
              nextBmpPressure >= 300.0f &&
              nextBmpPressure <= 1200.0f;
    if (ok) {
      bmpTempC = nextBmpTemp;
      bmpPressureHPa = nextBmpPressure;
      bmpReadFailCount = 0;
      updated = true;
    } else {
      if (bmpReadFailCount < 255) bmpReadFailCount++;
      if (bmpReadFailCount >= i2cSensorFailThreshold) {
        bmpReady = false;
        Serial.println("BMP280 disabled after repeated I2C read failures");
      }
    }
  }

  return updated;
}

bool readMpuWhoAmI(uint8_t address, uint8_t &whoAmI) {
  Wire.beginTransmission(address);
  Wire.write(0x75);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(static_cast<int>(address), 1) != 1) {
    return false;
  }

  whoAmI = Wire.read();
  return (whoAmI == 0x68 || whoAmI == 0x70 || whoAmI == 0x71);
}

bool readMpuRawSample() {
  Wire.beginTransmission(mpuI2CAddress);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(static_cast<int>(mpuI2CAddress), 14) != 14) {
    return false;
  }

  int16_t ax = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
  int16_t ay = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
  int16_t az = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
  int16_t tempRaw = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
  int16_t gx = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
  int16_t gy = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
  int16_t gz = static_cast<int16_t>((Wire.read() << 8) | Wire.read());

  accelX = static_cast<float>(ax) / 4096.0f;
  accelY = static_cast<float>(ay) / 4096.0f;
  accelZ = static_cast<float>(az) / 4096.0f;
  gyroX = static_cast<float>(gx) / 65.5f;
  gyroY = static_cast<float>(gy) / 65.5f;
  gyroZ = static_cast<float>(gz) / 65.5f;

  if (mpuWhoAmI == 0x70 || mpuWhoAmI == 0x71) {
    mpuTempC = (static_cast<float>(tempRaw) / 333.87f) + 21.0f;
  } else {
    mpuTempC = (static_cast<float>(tempRaw) / 340.0f) + 36.53f;
  }

  return true;
}

void sendTelemetryLine(const char *line) {
  if (!commandCharacteristic || !bluetoothConnected) return;
  commandCharacteristic->setValue(line);
  commandCharacteristic->notify();
}

void sendTelemetryPackets() {
  char line[32];

  snprintf(line, sizeof(line), "D,%02u%02u%04u,%02u%02u%02u\n",
           rtcDay, rtcMonth, rtcYear, rtcHour, rtcMinute, rtcSecond);
  sendTelemetryLine(line);

  snprintf(line, sizeof(line), "P,%d,%d,%d,%d,%d\n",
           commandToPercent(currentAngle1),
           commandToPercent(currentAngle2),
           bluetoothConnected ? 1 : 0,
           pumpRunning ? 1 : 0,
           servo1Sweeping ? 1 : 0);
  sendTelemetryLine(line);

  snprintf(line, sizeof(line), "L,%d\n", autoLevelEnabled ? 1 : 0);
  sendTelemetryLine(line);

  snprintf(line, sizeof(line), "B,%d\n", batteryVoltage);
  sendTelemetryLine(line);

  snprintf(line, sizeof(line), "A,%d,%d\n",
           static_cast<int>(ahtTempC * 10.0f),
           static_cast<int>(ahtHum * 10.0f));
  sendTelemetryLine(line);

  snprintf(line, sizeof(line), "R,%d,%d\n",
           static_cast<int>(bmpTempC * 10.0f),
           static_cast<int>(bmpPressureHPa));
  sendTelemetryLine(line);

  snprintf(line, sizeof(line), "M,%d,%d,%d\n",
           static_cast<int>(mpuTempC * 10.0f),
           static_cast<int>(rollDeg),
           static_cast<int>(pitchDeg));
  sendTelemetryLine(line);
}

void initRTC() {
  rtc.Begin();

  if (rtc.GetIsWriteProtected()) {
    rtc.SetIsWriteProtected(false);
  }

  if (!rtc.GetIsRunning()) {
    rtc.SetIsRunning(true);
  }

  if (!rtc.IsDateTimeValid()) {
    rtc.SetDateTime(RtcDateTime(__DATE__, __TIME__));
  }

  RtcDateTime now = rtc.GetDateTime();
  rtcReady = now.IsValid();
  if (rtcReady) {
    Serial.println("DS1302 ready");
  } else {
    Serial.println("DS1302 not ready");
  }
}

bool readRTC() {
  static unsigned long lastReadMs = 0;
  static uint8_t lastSecond = 255;
  static uint8_t lastMinute = 255;
  static uint8_t lastHour = 255;
  static uint8_t lastDay = 255;
  static uint8_t lastMonth = 255;
  static uint16_t lastYear = 0;
  const unsigned long readIntervalMs = 200;
  if ((millis() - lastReadMs) < readIntervalMs) return false;
  lastReadMs = millis();

  RtcDateTime now = rtc.GetDateTime();
  if (!now.IsValid()) {
    rtcReady = false;
    return false;
  }

  rtcReady = true;
  rtcYear = now.Year();
  rtcMonth = now.Month();
  rtcDay = now.Day();
  rtcHour = now.Hour();
  rtcMinute = now.Minute();
  rtcSecond = now.Second();

  bool changed = (rtcSecond != lastSecond) ||
                 (rtcMinute != lastMinute) ||
                 (rtcHour != lastHour) ||
                 (rtcDay != lastDay) ||
                 (rtcMonth != lastMonth) ||
                 (rtcYear != lastYear);

  lastSecond = rtcSecond;
  lastMinute = rtcMinute;
  lastHour = rtcHour;
  lastDay = rtcDay;
  lastMonth = rtcMonth;
  lastYear = rtcYear;

  return changed;
}
