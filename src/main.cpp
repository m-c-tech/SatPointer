#include <Arduino.h>
#include <Servo.h>
#include <SoftwareSerial.h>

#define debugSerial_TX 17
#define debugSerial_RX 18

EspSoftwareSerial::UART debugSerial;


float azimuth = 0.0;  // Store the current azimuth
float elevation = 0.0; // Store the current elevation
String inputString = "";      // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

static const int servoPin = 16;
Servo servo1;

void setup()
{
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
  inputString.reserve(200);
  servo1.attach(
      servoPin,
      Servo::CHANNEL_NOT_ATTACHED,
      0,
      120);
}

void runStepper()
{
  for (int posDegrees = 0; posDegrees <= 180; posDegrees++)
  {
    servo1.write(posDegrees);
    Serial.println(posDegrees);
    delay(20);
  }

  for (int posDegrees = 180; posDegrees >= 0; posDegrees--)
  {
    servo1.write(posDegrees);
    Serial.println(posDegrees);
    delay(20);
  }
}

void loop() {
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
        servo1.write(round(azimuth));
        debugSerial.print("Command:AZ");
        debugSerial.println(azimuth, 1);
      } else {
        Serial.print("AZ");
        Serial.println(azimuth, 1);
        debugSerial.print("AZ");
        debugSerial.println(azimuth, 1);
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