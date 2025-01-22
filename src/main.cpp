#include <Arduino.h>
#include <Servo.h>
#include <Adafruit_BNO08x.h>
#include <QuickPID.h>
#include <AccelStepper.h>

#define Serial2_TX 17
#define Serial2_RX -1
#define BNO08X_SDA 21
#define BNO08X_SCL 22
#define BNO08X_RESET -1
#define EN_PIN           -1 // Enable
#define DIR_PIN          19 // Direction
#define STEP_PIN         18 // Step

float azimuth = 0.0;  // Store the current azimuth
float elevation = 0.0; // Store the current elevation
String inputString = "";      // A string to hold incoming data
bool hasBNO08X = false;
static const int servoPin = 16;
float outputServoAngle = 0;
int servoInterval = 100; // In milliseconds
int servoElevationAngle = 0;
double previousServoMillis = 0;

// PID variables
float sensorAzimuthAngle = 0, outputAzimuthPID = 0, desiredAzimuthAngle = 0, azimuthError = 0;
float sensorElevationAngle = 0, outputElevationPID = 0, desiredElevationAngle = 45, elevationError = 0;
float setpoint = 0;

// PID constants
float azKp = 8, azKi = 1, azKd = 0;
float elKp = 0.5, elKi = 0, elKd = 0;


// PID objects
QuickPID azimuthPID(&azimuthError, &outputAzimuthPID, &setpoint, azKp, azKi, azKd,  /* OPTIONS */
               azimuthPID.pMode::pOnError,                   /* pOnError, pOnMeas, pOnErrorMeas */
               azimuthPID.dMode::dOnMeas,                    /* dOnError, dOnMeas */
               azimuthPID.iAwMode::iAwCondition,             /* iAwCondition, iAwClamp, iAwOff */
               azimuthPID.Action::reverse);
            
QuickPID elevationPID(&elevationError, &outputElevationPID, &setpoint, elKp, elKi, elKd,  /* OPTIONS */
               azimuthPID.pMode::pOnError,                   /* pOnError, pOnMeas, pOnErrorMeas */
               azimuthPID.dMode::dOnMeas,                    /* dOnError, dOnMeas */
               azimuthPID.iAwMode::iAwCondition,             /* iAwCondition, iAwClamp, iAwOff */
               azimuthPID.Action::reverse);

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

AccelStepper stepper1(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

Servo servo1;
Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
sh2_SensorId_t reportType = SH2_ROTATION_VECTOR;
long reportIMUIntervalUs = 100000;

void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial2.println("Setting desired reports");
  if (!bno08x.enableReport(reportType, report_interval)) {
    Serial2.println("Could not enable stabilized remote vector");
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, Serial2_RX, Serial2_TX, false);
  if (!bno08x.begin_I2C()) {
    Serial2.println("Failed to find BNO08x chip");
  } else {
    Serial2.println("BNO08x started");
    hasBNO08X = true;
    setReports(reportType, reportIMUIntervalUs);
  }
  azimuthPID.SetOutputLimits(-1000, 1000);
  azimuthPID.SetSampleTimeUs(100000);
  azimuthPID.SetTunings(azKp, azKi, azKd);
  azimuthPID.SetMode(azimuthPID.Control::automatic);
  elevationPID.SetOutputLimits(-20, 20);
  elevationPID.SetSampleTimeUs(100000);
  elevationPID.SetTunings(elKp, elKi, elKd);
  elevationPID.SetMode(elevationPID.Control::automatic);
  inputString.reserve(200);
  servo1.attach(servoPin, Servo::CHANNEL_NOT_ATTACHED, 0, 180);  
  stepper1.setMaxSpeed(1000);
  stepper1.setEnablePin(EN_PIN);
  stepper1.setPinsInverted(true, true, false);
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

void processEasycommCommands(String inputString) {
  //Process Easycomm II rotator commands
  //Easycomm II position command: AZnn.n ELnn.n UP000 XXX DN000 XXX\n
  //Easycomm II query command: AZ EL \n
  String param;                                           //Parameter value
  int firstSpace;                                         //Position of the first space in the command inputString
  int secondSpace;                                        //Position of the second space in the command inputString
  if (inputString.startsWith("AZ") && (inputString.length() == 2)) {                         //Query command received
    Serial.print("AZ");
    Serial.println(sensorAzimuthAngle, 1);
    Serial2.print("AZ");
    Serial2.print(sensorAzimuthAngle, 1); Serial2.print("\t");                                          //Send the current Azimuth and Elevation
  } else {
    if (inputString.startsWith("AZ")) {                          //Position command received: Parse the inputString.
      firstSpace = inputString.indexOf(' ');                     //Get the position of the first space
      secondSpace = inputString.indexOf(' ', firstSpace + 1);    //Get the position of the second space
      param = inputString.substring(2, firstSpace);              //Get the first parameter
      desiredAzimuthAngle = param.toFloat();                            //Set the azSet value
      Serial2.print("Command:AZ"); Serial2.print(param); Serial2.print("\t");
      param = inputString.substring(firstSpace + 3, secondSpace);//Get the second parameter
      desiredElevationAngle = param.toFloat();                            //Set the elSet value
      Serial2.print("Command:EL"); Serial2.println(param);
    }
  }
  if (inputString.startsWith("EL") && (inputString.length() == 2)) {                         //Query command received
    Serial.print("EL");
    Serial.println(sensorElevationAngle, 1);
    Serial2.print("EL");
    Serial2.println(sensorElevationAngle, 1);                                          //Send the current Azimuth and Elevation
  }
  inputString = "";
}

float computeShortestAngleDifference(float targetAngle, float currentAngle) {
  float difference = targetAngle - currentAngle;

  // Wrap the difference to the range -180 to 180
  while (difference > 180) difference -= 360;
  while (difference < -180) difference += 360;

  return difference;
}

void processCommands(void) {
  //Process incoming data from the control computer
  //User commands are entered by the user and are terminated with a carriage return
  //Easycomm commands are generated by a tracking program and are terminated with a line feed
  while (Serial.available()) {
    char ch = Serial.read();                                //Read a single character from the serial buffer
    switch (ch) {
      case 13:                                                  //Carriage return received
        //processUserCommands(inputString);                              //Process user commands
        inputString = "";                                              //Command processed: Clear the command line
        break;
      case 10:                                                  //Line feed received
        processEasycommCommands(inputString);                          //Process Easycomm commands
        inputString = "";                                              //Command processed: Clear the command line
        break;
      default:                                                  //Any other character received
        inputString += ch;                                             //Add this character to the command line
        break;
    }
  }
}

void readSensorData() {
  quaternionToEulerRV(&sensorValue.un.rotationVector, &ypr, true);
  sensorAzimuthAngle = map(ypr.yaw * -1, -180, 180, 0, 360);
  sensorElevationAngle = (ypr.pitch * -1);
}

void loop() {
  if (bno08x.wasReset()) {
    Serial2.print("sensor was reset ");
    setReports(reportType, reportIMUIntervalUs);
  }
  if (bno08x.getSensorEvent(&sensorValue)) {
    readSensorData();
  }

  processCommands();

  azimuthError = computeShortestAngleDifference(desiredAzimuthAngle, sensorAzimuthAngle);
  azimuthPID.Compute();
  stepper1.setSpeed(outputAzimuthPID);
  stepper1.runSpeed();
  //Serial2.print("sensorAzimuthAngle:");   Serial2.print(ypr.yaw*-1);       Serial2.print("\t"); //Azimuth
  //Serial2.print("outputAzimuthPID:"); Serial2.print(outputAzimuthPID); Serial2.print("\t");
  //Serial2.print("azimuthError"); Serial2.print(azimuthError); Serial2.println("\t");
  
  if (millis() - previousServoMillis > servoInterval) {
    elevationError = computeShortestAngleDifference(desiredElevationAngle, sensorElevationAngle);
    elevationPID.Compute();
    servoElevationAngle = constrain(servoElevationAngle + outputElevationPID, 0 , 90);
    servo1.write(servoElevationAngle);
    previousServoMillis = millis();
  }

  //Serial2.print("elevationError:"); Serial2.print(elevationError);  Serial2.print("\t"); //Elevation 
  //Serial2.print("desiredElevationAngle:"); Serial2.print(desiredElevationAngle); Serial2.print("\t");
  //Serial2.print("outputElevationPID:"); Serial2.println(outputElevationPID); Serial2.print("\t");
  //Serial2.print("servoElevationAngle:"); Serial2.print(servoElevationAngle); Serial2.print("\t");
}



