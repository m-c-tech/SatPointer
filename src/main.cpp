#include <Arduino.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <Adafruit_BNO08x.h>
#include <QuickPID.h>

#define debugSerial_TX 17
#define debugSerial_RX 18
#define BNO08X_SDA 21
#define BNO08X_SCL 22
#define BNO08X_RESET -1

float azimuth = 0.0;  // Store the current azimuth
float elevation = 0.0; // Store the current elevation
String inputString = "";      // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete
bool hasBNO08X = false;
static const int servoPin = 16;
float outputServoAngle = 0;
float desiredAzimuthAngle = 45, sensorAzimuthAngle = 0, outputAzimuthPID = 0, Kp = 0.1, Ki = 0.8, Kd = 0;

QuickPID azimuthPID(&sensorAzimuthAngle, &outputAzimuthPID, &desiredAzimuthAngle, Kp, Ki, Kd,  /* OPTIONS */
               azimuthPID.pMode::pOnError,                   /* pOnError, pOnMeas, pOnErrorMeas */
               azimuthPID.dMode::dOnMeas,                    /* dOnError, dOnMeas */
               azimuthPID.iAwMode::iAwCondition,             /* iAwCondition, iAwClamp, iAwOff */
               azimuthPID.Action::direct); 

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

Servo servo1;
EspSoftwareSerial::UART debugSerial;
Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
sh2_SensorId_t reportType = SH2_ROTATION_VECTOR;
long reportIMUIntervalUs = 10000;

void setReports(sh2_SensorId_t reportType, long report_interval) {
  debugSerial.println("Setting desired reports");
  if (!bno08x.enableReport(reportType, report_interval)) {
    debugSerial.println("Could not enable stabilized remote vector");
  }
}

void setup() {
  Serial.begin(115200);
  debugSerial.begin(38400, SWSERIAL_8N1, debugSerial_RX, debugSerial_TX, false);
  if (!debugSerial) { // If the object did not initialize, then its configuration is invalid
    Serial.println("Invalid EspSoftwareSerial pin configuration, check config"); 
    while (1) { // Don't continue with invalid configuration
      delay (1000);
    }
  } else {
    debugSerial.println("Debug serial started");
  }
  if (!bno08x.begin_I2C()) {
    debugSerial.println("Failed to find BNO08x chip");
  } else {
    debugSerial.println("BNO08x started");
    hasBNO08X = true;
    setReports(reportType, reportIMUIntervalUs);
  }
  azimuthPID.SetOutputLimits(0, 180);
  azimuthPID.SetSampleTimeUs(100000);
  azimuthPID.SetTunings(Kp, Ki, Kd);
  azimuthPID.SetMode(azimuthPID.Control::automatic);
  inputString.reserve(200);
  servo1.attach(servoPin, Servo::CHANNEL_NOT_ATTACHED, 0, 180);
}

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void loop() {
  if (bno08x.wasReset()) {
    debugSerial.print("sensor was reset ");
    setReports(reportType, reportIMUIntervalUs);
  }

  if (bno08x.getSensorEvent(&sensorValue)) {
    quaternionToEulerRV(&sensorValue.un.rotationVector, &ypr, true);
    debugSerial.print(ypr.roll*-1);                       debugSerial.print("\t"); //Elevation 
    debugSerial.print(map(ypr.yaw, 180, -180, 0, 360));   debugSerial.println("\t"); //Azimuth
    sensorAzimuthAngle = (ypr.roll*-1);
  }

  azimuthPID.Compute();
  servo1.write(outputAzimuthPID);

  if (stringComplete) {
    // Remove leading spaces and check the input
    inputString.trim();
    if (inputString.startsWith("AZ")) {
      // If "AZ" is received
      inputString.remove(0, 2); // Remove "AZ" from the start
      // Check if there's a float number following "AZ"
      if (inputString.length() > 0) {
        azimuth = inputString.toFloat(); // Convert the remaining string to a float
        Serial.print("AZ");
        Serial.println(azimuth, 1);
        desiredAzimuthAngle = azimuth;
        debugSerial.print("Command:AZ");
        debugSerial.println(azimuth, 1);
      } else {
        Serial.print("AZ");
        Serial.println(azimuth, 1);
        debugSerial.print("AZ");
        debugSerial.println(sensorAzimuthAngle, 1);
      }
    } else if (inputString.startsWith("EL")) {
      // If "EL" is received
      inputString.remove(0, 2); // Remove "EL" from the start
      
      // Check if there's a float number following "EL"
      if (inputString.length() > 0) {
        elevation = inputString.toFloat(); // Convert the remaining string to a float
        Serial.print("EL");
        Serial.println(elevation, 1);
        debugSerial.print("Command:EL");
        debugSerial.println(elevation, 1);
      } else {
        Serial.print("EL");
        Serial.println(elevation, 1);
        debugSerial.print("EL");
        debugSerial.println(elevation, 1);
      }
    } else {
      // If the input doesn't match AZ or EL commands
      debugSerial.println("Unknown command:");
      debugSerial.println(inputString);
    }
    
    // Reset the inputString for the next command
    inputString = "";
    stringComplete = false;
  }
}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}