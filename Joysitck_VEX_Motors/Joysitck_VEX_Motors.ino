#include <ESP32_Servo.h>
static const int motorPins[10] = {13, 15, 2, 4, 16, 12, 14, 21, 22, 25};
Servo Apollo_motors[10];

// Remote Control Input
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;
//              0  1  2  3   4   5   6   7   8   9  10  11  12  13  14  15  16  17     18      19
//              A, B, Y, X, UP, DN, LT, RT, L1, L2, L3, R1, R2, R3, LX, LY, RX, RY, Start, Select;
int remote[] = {0, 0, 0, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,     0,      0};
boolean stringComplete = false;
String inputString = "";
char tempChar[5];

// Speed is 0-100
void driveMotors(int left_speed, int right_speed) {
  int left_value = (-left_speed * 90 / 50);
  int right_value = (right_speed * 90 / 50);
  Apollo_motors[0].write(left_value + 90);
  Apollo_motors[1].write(left_value + 90);
  Apollo_motors[2].write(right_value + 90);
  Apollo_motors[3].write(right_value + 90);
}

long lastMessage = 0;
int counter = 0;
bool lastState = false;
boolean firstAutoLoop = true;
void readRemoteInput() {
  // Check the buffer for data
  while (SerialBT.available()) {
    char inChar = (char)SerialBT.read();
    // Parse the message
    if (inChar == '.') {     // Handle the end of the message
      stringComplete = true;
      counter = 0;
    }
    else if (inChar == ',') { // Each time a comma is hit
      inputString.toCharArray(tempChar, 5);
      remote[counter] = atoi(tempChar);
      Serial.print(counter);
      Serial.print(" : ");
      Serial.println(remote[counter]);
      inputString = "";
      counter++;
    }
    else {
      inputString += inChar;
    }
  }

  // Each Time there is a new message, this will trigger
  if (stringComplete) {
    lastMessage = millis();
    stringComplete = false;
  }
}

#include "esp_system.h"
String getMacAddress() {
  uint8_t baseMac[6];
  // Get MAC address for WiFi station
  esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
  char baseMacChr[18] = {0};
  sprintf(baseMacChr, "%02X%02X%02X", baseMac[3], baseMac[4], baseMac[5]);
  return String(baseMacChr);
}

void setup() {
  Serial.begin(115200);

  String buf;
  buf += "Atlas-";
  buf += getMacAddress();
  Serial.println(buf);
  SerialBT.begin(buf);

  // Set up all Motor ports on the Apollo
  for (int i = 0; i < 10; ++i) {
    if (!Apollo_motors[i].attach(motorPins[i], 1000, 2000)) {
      Serial.print("Motor ");
      Serial.print(i);
      Serial.println("attach error");
    }
  }
  // Built in LED
  pinMode(32, OUTPUT);
  digitalWrite(32, HIGH);
}

int analogPosition = 0;

void loop() {
  // Check the Remote input
  readRemoteInput();
  // Make sure that the remote is actively connected to the robot
  if (millis() - lastMessage < 1000) {
    // Command the drive motors based on left/right commands
    driveMotors(remote[15], remote[17]);

    // Command Motor #5
    // Turn off when X is pressed
    if (remote[3] == 1) {
      digitalWrite(32, HIGH);
      Apollo_motors[4].write(90);
    }
    // Turn on when Y is pressed
    if (remote[2] == 1) {
      digitalWrite(32, LOW);
      Apollo_motors[4].write(45);
    }

    // Hold B down and adjust trigger/slider.  Release B to hold that position/speed.
    if (remote[1] == 1) {
      analogPosition = remote[12];
      SerialBT.println(analogPosition);
    }
    // Command Motor forward based on the shoulder analog slider
    Apollo_motors[5].write(90 + (analogPosition * 90 / 100));
  }
  else {
    // If disconnected, stop all motors
    driveMotors(0, 0);
    Apollo_motors[4].write(90);
    Apollo_motors[5].write(90);
  }
}
