
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

// Remote Control Input
//              0  1  2  3   4   5   6   7   8   9  10  11  12  13  14  15  16  17     18      19
//              A, B, Y, X, UP, DN, LT, RT, L1, L2, L3, R1, R2, R3, LX, LY, RX, RY, Start, Select;
int remote[] = {0, 0, 0, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,     0,      0};
boolean stringComplete = false;
String inputString = "";
char tempChar[5];

#include "esp_bt_main.h"
#include "esp_bt_device.h"

int Right_1 = 13;   // M1
int Right_2 = 15;   // M2
int Right_EN = 4;   // M3
int Left_1 = 21;   // M8
int Left_2 = 22;   // M9
int Left_EN = 25;   // M10
// setting PWM properties
const int freq = 50000;
const int ledChannel = 0, ledChannel2 = 1;
const int resolution = 8;

#define GET_LOW_BYTE(A) (uint8_t)((A))
#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8)
#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))

#define LOBOT_SERVO_FRAME_HEADER         0x55
#define LOBOT_SERVO_MOVE_TIME_WRITE      1
#define LOBOT_SERVO_MOVE_TIME_READ       2
#define LOBOT_SERVO_MOVE_TIME_WAIT_WRITE 7
#define LOBOT_SERVO_MOVE_TIME_WAIT_READ  8
#define LOBOT_SERVO_MOVE_START           11
#define LOBOT_SERVO_MOVE_STOP            12
#define LOBOT_SERVO_ID_WRITE             13
#define LOBOT_SERVO_ID_READ              14
#define LOBOT_SERVO_ANGLE_OFFSET_ADJUST  17
#define LOBOT_SERVO_ANGLE_OFFSET_WRITE   18
#define LOBOT_SERVO_ANGLE_OFFSET_READ    19
#define LOBOT_SERVO_ANGLE_LIMIT_WRITE    20
#define LOBOT_SERVO_ANGLE_LIMIT_READ     21
#define LOBOT_SERVO_VIN_LIMIT_WRITE      22
#define LOBOT_SERVO_VIN_LIMIT_READ       23
#define LOBOT_SERVO_TEMP_MAX_LIMIT_WRITE 24
#define LOBOT_SERVO_TEMP_MAX_LIMIT_READ  25
#define LOBOT_SERVO_TEMP_READ            26
#define LOBOT_SERVO_VIN_READ             27
#define LOBOT_SERVO_POS_READ             28
#define LOBOT_SERVO_OR_MOTOR_MODE_WRITE  29
#define LOBOT_SERVO_OR_MOTOR_MODE_READ   30
#define LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE 31
#define LOBOT_SERVO_LOAD_OR_UNLOAD_READ  32
#define LOBOT_SERVO_LED_CTRL_WRITE       33
#define LOBOT_SERVO_LED_CTRL_READ        34
#define LOBOT_SERVO_LED_ERROR_WRITE      35
#define LOBOT_SERVO_LED_ERROR_READ       36

//#define LOBOT_DEBUG 0  /*Debug ：print debug value*/

byte LobotCheckSum(byte buf[])
{
  byte i;
  uint16_t temp = 0;
  for (i = 2; i < buf[3] + 2; i++) {
    temp += buf[i];
  }
  temp = ~temp;
  i = (byte)temp;
  return i;
}
void LobotSerialServoMove(HardwareSerial &SerialX, uint8_t id, int16_t position, uint16_t time)
{
  byte buf[10];
  if (position < 0)
    position = 0;
  if (position > 1000)
    position = 1000;
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = LOBOT_SERVO_MOVE_TIME_WRITE;
  buf[5] = GET_LOW_BYTE(position);
  buf[6] = GET_HIGH_BYTE(position);
  buf[7] = GET_LOW_BYTE(time);
  buf[8] = GET_HIGH_BYTE(time);
  buf[9] = LobotCheckSum(buf);
  SerialX.write(buf, 10);
}
void LobotSerialServoStopMove(HardwareSerial &SerialX, uint8_t id)
{
  byte buf[6];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_MOVE_STOP;
  buf[5] = LobotCheckSum(buf);
  SerialX.write(buf, 6);
}
void LobotSerialServoSetID(HardwareSerial &SerialX, uint8_t oldID, uint8_t newID)
{
  byte buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = oldID;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_ID_WRITE;
  buf[5] = newID;
  buf[6] = LobotCheckSum(buf);
  SerialX.write(buf, 7);

#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO ID WRITE");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

}
void LobotSerialServoMOVE(HardwareSerial &SerialX, uint8_t id, uint8_t newID)
{
  byte buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_ID_WRITE;
  buf[5] = newID;
  buf[6] = LobotCheckSum(buf);
  SerialX.write(buf, 7);
}
void LobotSerialServoSetMode(HardwareSerial &SerialX, uint8_t id, uint8_t Mode, int16_t Speed)
{
  byte buf[10];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = LOBOT_SERVO_OR_MOTOR_MODE_WRITE;
  buf[5] = Mode;
  buf[6] = 0;
  buf[7] = GET_LOW_BYTE((uint16_t)Speed);
  buf[8] = GET_HIGH_BYTE((uint16_t)Speed);
  buf[9] = LobotCheckSum(buf);

#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO Set Mode");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

  SerialX.write(buf, 10);
}
void LobotSerialServoLoad(HardwareSerial &SerialX, uint8_t id)
{
  byte buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE;
  buf[5] = 1;
  buf[6] = LobotCheckSum(buf);

  SerialX.write(buf, 7);

#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO LOAD WRITE");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

}
void LobotSerialServoUnload(HardwareSerial &SerialX, uint8_t id)
{
  byte buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE;
  buf[5] = 0;
  buf[6] = LobotCheckSum(buf);

  SerialX.write(buf, 7);

#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO LOAD WRITE");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif
}
int LobotSerialServoReceiveHandle(HardwareSerial &SerialX, byte *ret)
{
  bool frameStarted = false;
  bool receiveFinished = false;
  byte frameCount = 0;
  byte dataCount = 0;
  byte dataLength = 2;
  byte rxBuf;
  byte recvBuf[32];
  byte i;

  while (SerialX.available()) {
    rxBuf = SerialX.read();
    delayMicroseconds(100);
    if (!frameStarted) {
      if (rxBuf == LOBOT_SERVO_FRAME_HEADER) {
        frameCount++;
        if (frameCount == 2) {
          frameCount = 0;
          frameStarted = true;
          dataCount = 1;
        }
      }
      else {
        frameStarted = false;
        dataCount = 0;
        frameCount = 0;
      }
    }
    if (frameStarted) {
      recvBuf[dataCount] = (uint8_t)rxBuf;
      if (dataCount == 3) {
        dataLength = recvBuf[dataCount];
        if (dataLength < 3 || dataCount > 7) {
          dataLength = 2;
          frameStarted = false;
        }
      }
      dataCount++;
      if (dataCount == dataLength + 3) {

#ifdef LOBOT_DEBUG
        Serial.print("RECEIVE DATA:");
        for (i = 0; i < dataCount; i++) {
          Serial.print(recvBuf[i], HEX);
          Serial.print(":");
        }
        Serial.println(" ");
#endif

        if (LobotCheckSum(recvBuf) == recvBuf[dataCount - 1]) {

#ifdef LOBOT_DEBUG
          Serial.println("Check SUM OK!!");
          Serial.println("");
#endif

          frameStarted = false;
          memcpy(ret, recvBuf + 4, dataLength);
          return 1;
        }
        return -1;
      }
    }
  }
}
int LobotSerialServoReadPosition(HardwareSerial &SerialX, uint8_t id)
{
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_POS_READ;
  buf[5] = LobotCheckSum(buf);

#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO Pos READ");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

  while (SerialX.available())
    SerialX.read();

  SerialX.write(buf, 6);

  while (!SerialX.available()) {
    count -= 1;
    if (count < 0)
      return -2048;
  }

  if (LobotSerialServoReceiveHandle(SerialX, buf) > 0)
    ret = (int16_t)BYTE_TO_HW(buf[2], buf[1]);
  else
    ret = -2048;

#ifdef LOBOT_DEBUG
  Serial.println(ret);
#endif
  return ret;
}
int LobotSerialServoReadVin(HardwareSerial &SerialX, uint8_t id)
{
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_VIN_READ;
  buf[5] = LobotCheckSum(buf);

#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO VIN READ");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

  while (SerialX.available())
    SerialX.read();

  SerialX.write(buf, 6);

  while (!SerialX.available()) {
    count -= 1;
    if (count < 0)
      return -2048;
  }

  if (LobotSerialServoReceiveHandle(SerialX, buf) > 0)
    ret = (int16_t)BYTE_TO_HW(buf[2], buf[1]);
  else
    ret = -2049;

#ifdef LOBOT_DEBUG
  Serial.println(ret);
#endif
  return ret;
}

#define ID1   1
#define ID2   2
HardwareSerial smartMotor(2);

// LED
#define NUM_LEDS 7
#include "ws2812_control.h"
#define GREEN   0xFF0000
#define RED 0x00FF00
#define BLUE  0x0000FF
struct led_state new_state;

// Speed is 0-100
void driveMotors(int left_speed, int right_speed) {
  left_speed = - left_speed;
  Serial.print("LEFT: ");
  if (left_speed > 0) {
    digitalWrite (Left_2, HIGH);
    digitalWrite (Left_1, LOW);
    ledcWrite(ledChannel2, map(left_speed, 100, 0, 245, 120));
    Serial.print(map(left_speed, 100, 0, 245, 120));
  }
  else if (left_speed < 0) {
    digitalWrite (Left_2, LOW);
    digitalWrite (Left_1, HIGH);
    ledcWrite(ledChannel2, map(left_speed, -100, 0, 245, 120));
    Serial.print(map(left_speed, -100, 0, 245, 120));
  }
  else {
    ledcWrite(ledChannel2, 0);
  }
  Serial.print("\tRIGHT: ");
  if (right_speed < 0) {
    digitalWrite (Right_2, HIGH);
    digitalWrite (Right_1, LOW);
    ledcWrite(ledChannel, map(right_speed, -100, 0, 245, 120));
    Serial.print(map(right_speed, -100, 0, 245, 120));
  }
  else if (right_speed > 0) {
    digitalWrite (Right_2, LOW);
    digitalWrite (Right_1, HIGH);
    ledcWrite(ledChannel,map(right_speed, 100, 0, 245, 120) );
    Serial.print(map(right_speed, 100, 0, 245, 120));
  }
  else {
    ledcWrite(ledChannel, 0);
  }
  Serial.println ("");
}

int currentShoulder = 0;
void setup() {
  SerialBT.begin("Atlas-2");
  // put your setup code here, to run once:
  Serial.begin(115200);

  smartMotor.begin(115200, SERIAL_8N1, 16, 17);
  //Serial.println("Starting...");
  //delay(2000);
  Serial.println(LobotSerialServoReadVin(smartMotor, 2));
  Serial.println(LobotSerialServoReadPosition(smartMotor, 2));

  //LobotSerialServoMove(smartMotor, ID2, 1, 500);
  //delay(500);
  //LobotSerialServoMove(Serial2, ID2, 1, 1000);
  //delay(500);
  LobotSerialServoSetMode(smartMotor, ID2, 0, LobotSerialServoReadPosition(smartMotor, ID2)); // Set ID2 to Servo mode at a position of 500
  int pose = LobotSerialServoReadPosition(smartMotor, ID1);
  if (pose > -1000) {
    currentShoulder = pose;
  }
  else {
    currentShoulder = 500;
  }
  Serial.println("Second");

  pinMode(32, OUTPUT);
  pinMode(33, INPUT_PULLUP);

  pinMode (Right_EN, OUTPUT);
  pinMode (Right_1, OUTPUT);
  pinMode (Right_2, OUTPUT);
  pinMode (Left_EN, OUTPUT);
  pinMode (Left_1, OUTPUT);
  pinMode (Left_2, OUTPUT);

  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(Right_EN, ledChannel);
  ledcWrite(ledChannel, 100);

  ledcSetup(ledChannel2, freq, resolution);
  ledcAttachPin(Left_EN, ledChannel2);
  ledcWrite(ledChannel2, 100);

  pinMode(27, INPUT_PULLUP);
  ws2812_control_init();
  new_state.leds[0] = GREEN;

  ws2812_write_leds(new_state);
}
int pos = 0;

// Rotational control of the motor
// LobotSerialServoSetMode(smartMotor, ID2, 1, speed);(-1000...1000)

int potSpot = 0;

long lastMessage = 0;
int counter = 0;
bool lastState = false;
boolean firstAutoLoop = true;
void readRemoteInput() {
  //Serial.println("reading");
  while (SerialBT.available()) {
    char inChar = (char)SerialBT.read();
    //Serial.println("incoming");
    if (inChar == '.') {     // Handle the end of the message
      stringComplete = true;
    }
    else if (inChar == ',') { // Each time a comma is hit
      inputString.toCharArray(tempChar, 5);
      remote[counter] = atoi(tempChar);
      //Serial.print(counter);
      //Serial.print(" : ");
      //Serial.println(remote[counter]);
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
    counter = 0;
  }
}

int last_OUT_message = 0;
void loop() {
  readRemoteInput();
  if (millis() - lastMessage > 1000) {
    ledcWrite(ledChannel, 0);
    ledcWrite(ledChannel2, 0);
  }

  if (millis() - lastMessage < 1000) {
    // Command the drive motors based on left/right commands
    driveMotors(remote[15], remote[17]);

    // Command Motor #5
    // Turn off when X is pressed
    if (remote[3] == 1) {
      //digitalWrite(32, HIGH);
      LobotSerialServoMove(smartMotor, ID2, 343, 1);
    }
    // Turn on when Y is pressed
    if (remote[2] == 1) {
      //digitalWrite(32, LOW);
      LobotSerialServoMove(smartMotor, ID2, 467, 1);
    }

    // Hold B down and adjust trigger/slider.  Release B to hold that position/speed.
    // Command Motor forward based on the shoulder analog slider
    potSpot = map(remote[12], 0, 100, 316, 730);
    //Serial.print("Pot : ");
    //Serial.println(potSpot);
    //Serial.print("Current : ");
    //Serial.println(currentShoulder);
    if (potSpot - currentShoulder > 10) {
      currentShoulder = currentShoulder + 10;
    }
    else if (currentShoulder - potSpot > 10) {
      currentShoulder = currentShoulder - 10;
    }
    else {
      currentShoulder = potSpot;
    }
    LobotSerialServoMove(smartMotor, ID1, currentShoulder, 100);
  }

  if (digitalRead(27) == HIGH) {
    //digitalWrite(32, HIGH);
    new_state.leds[0] = RED;
    ws2812_write_leds(new_state);
  }
  else {
    //digitalWrite(32, LOW);
    new_state.leds[0] = BLUE;
    ws2812_write_leds(new_state);
  }
}
