#include <ArduinoJson.h>

#include <MeMegaPi.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include "MeGyro.h"
#include "MeUltrasonicSensor.h"
#include "MeLineFollower.h"

#define ENC_MOTOR_GEAR_RATIO 46.67
#define REG_MOTOR_GEAR_RATIO 75

#define DRIVE_RPM 255 //full speed
#define STOP_RPM 0


//Encoder Motor
MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
MeEncoderOnBoard Encoder_3(SLOT3);
MeGyro Gyro(PORT_6); //CHECK PORT
MeUltrasonicSensor UltraSensor(PORT_8); //CHECK PORT
MeLineFollower LineFinder(PORT_7);      //CHECK PORT
//MeBluetooth Bluetooth(PORT_5);          //CHECK PORT
StaticJsonBuffer<1000> jsonBufferOut;
StaticJsonBuffer<512> jsonBufferIn;
char in[512] = {'\0'};

//Math stuff
double angle_rad = PI / 180.0;
double angle_deg = 180.0 / PI;

// Very important
bool LOCKED_STATE = false;
float DIST_RATIO = 1.2;   //CHANGE THIS NUMBER
int ANGLE_INCR = 5;       //CHANGE THIS NUMBER



void isr_process_encoder1(void)
{
  if (digitalRead(Encoder_1.getPortB()) == 0) {
    Encoder_1.pulsePosMinus();
  } else {
    Encoder_1.pulsePosPlus();
  }
}

void isr_process_encoder2(void)
{
  if (digitalRead(Encoder_2.getPortB()) == 0) {
    Encoder_2.pulsePosMinus();
  } else {
    Encoder_2.pulsePosPlus();
  }
}

void isr_process_encoder3(void)
{
  if (digitalRead(Encoder_3.getPortB()) == 0) {
    Encoder_3.pulsePosMinus();
  } else {
    Encoder_3.pulsePosPlus();
  }
}

void move(int direction, int speed)
{
  int leftSpeed = 0;
  int rightSpeed = 0;
  if (direction == 1) {
    leftSpeed = -speed;
    rightSpeed = speed;
  } else if (direction == 2) {
    leftSpeed = speed;
    rightSpeed = -speed;
  } else if (direction == 3) {
    leftSpeed = speed;
    rightSpeed = speed;
  } else if (direction == 4) {
    leftSpeed = -speed;
    rightSpeed = -speed;
  }
  Encoder_1.setTarPWM(rightSpeed);
  Encoder_2.setTarPWM(leftSpeed);
}
void moveDegrees(int direction, long degrees, int speed_temp)
{
  speed_temp = abs(speed_temp);
  if (direction == 1)
  {
    Encoder_1.move(degrees, (float)speed_temp);
    Encoder_2.move(-degrees, (float)speed_temp);
  }
  else if (direction == 2)
  {
    Encoder_1.move(-degrees, (float)speed_temp);
    Encoder_2.move(degrees, (float)speed_temp);
  }
  else if (direction == 3)
  {
    Encoder_1.move(degrees, (float)speed_temp);
    Encoder_2.move(degrees, (float)speed_temp);
  }
  else if (direction == 4)
  {
    Encoder_1.move(-degrees, (float)speed_temp);
    Encoder_2.move(-degrees, (float)speed_temp);
  }

}

// return difference from desired to actaul
float go(float distance) {
  // GET START ANGLE FROM ENCODER
  Encoder_1.updateCurPos();
  long start_angle = Encoder_1.getCurPos(); //okay to only use 1?
  float rotations = distance / DIST_RATIO;
  float move_angle = rotations * 360.0;
  Encoder_1.move(move_angle);
  Encoder_2.move(move_angle);
  // GET END ANGLE FROM ENCODER
  Encoder_1.updateCurPos();
  return start_angle - Encoder_1.getCurPos();
}


// ADD SOME MORE MATH
// WILL CONTINUE ROTATING IF OVERSHOOT
//returns differences
float turn(float turn_angle) {
  // GET CURRENT ANGLE FROM GYRO
  Gyro.update();
  double start_angle = Gyro.getAngleZ();
  // DO MATH TO FIND DESTINATION
  float dest_angle = ((int)(start_angle + turn_angle)) % 360;
  double curr_angle = start_angle;
  while ( abs(dest_angle - curr_angle) > ANGLE_INCR ) {
    Encoder_1.move(ANGLE_INCR);
    Encoder_2.move(-ANGLE_INCR);
    // UPDATE CURRENT ANGLE
    Gyro.update();
    curr_angle = Gyro.getAngleZ();
  }
  return (start_angle - curr_angle);
}

void setup() {
//  Gyro.begin();
  Serial.begin(9600);
  Serial.print("Serial set up complete.");
  Serial3.begin(115200);
  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
  attachInterrupt(Encoder_3.getIntNum(), isr_process_encoder3, RISING);
  Encoder_1.setPulse(8);
  Encoder_2.setPulse(8);
  Encoder_3.setPulse(8);
  Encoder_1.setRatio(ENC_MOTOR_GEAR_RATIO); //186 RPM motor
  Encoder_2.setRatio(ENC_MOTOR_GEAR_RATIO); //186 RPM motor
  Encoder_3.setRatio(REG_MOTOR_GEAR_RATIO); // 86 RPM motor
  Encoder_1.setPosPid(1.8, 0, 1.2);
  Encoder_2.setPosPid(1.8, 0, 1.2);
  Encoder_3.setPosPid(1.8, 0, 1.2);
  Encoder_1.setSpeedPid(0.18, 0, 0);
  Encoder_2.setSpeedPid(0.18, 0, 0);
  Encoder_3.setSpeedPid(0.18, 0, 0);
  Serial.print("Set up complete.");
}

void loop() {
  if (Serial3.available()) {
    int i = 0;
    int data;
    while ((data = Serial3.read()) != (int)-1) {
      in[i] = data;
      i++;
    }
    parseCommand();

    memset(&in[0], '\0', sizeof(in));
  }
  //_loop();
  //sendData();
}

void parseCommand() {
  JsonObject& root = jsonBufferIn.parseObject(in);
  Serial.println("Parsing...");
  if (root.success() && !LOCKED_STATE) {
    int command = root["type"].as<int>();
    Serial.print("Type: ");
    Serial.print(command);
    switch (command) {
      case 0:
        {
          //LOCK
          break;
        }
      case 1:
        {
          //UNLOCK
          break;
        }
      case 2:
        {
          //KILL SWITCH
          break;
        }
      case 3:
        {
          //MODE CHANGE
          break;
        }
      case 4:
        {
          //MOVE
          bool forward = root["args"]["forward"].as<bool>();
          cmdForward(forward);
          break;
        }
      case 5:
        {
          //MOVE
          bool backward = root["args"]["backward"].as<bool>();
          cmdBackward(backward);
          break;
        }
      case 6:
        {
          //TURN LEFT
          bool left = root["args"]["left"].as<bool>();
          cmdTurnLeft(left);
          break;
        }
      case 7:
        {
          //TURN RIGHT
          bool right = root["args"]["right"].as<bool>();
          cmdTurnRight(right);
        }
      case 9:
        {
          //RAISE CRANE
          bool raiseCrane = root["args"]["raise"].as<bool>();
          cmdRaiseCrane(raiseCrane);
        }
      case 10:
        {
          //LOWER CRANE
          bool lowerCrane = root["args"]["lower"].as<bool>();
          cmdLowerCrane(lowerCrane);
        }
      case 12:
        {
          //OPEN CLAW
        }       
      case 13:
        {
          //CLOSE CLAW
        }
    }
  }
}


void cmdForward(bool forward) {
  if (forward) { //GO
    Encoder_1.runSpeed(DRIVE_RPM);
    Encoder_2.runSpeed(DRIVE_RPM);
  } else {      //STOP
    Encoder_1.runSpeed(STOP_RPM);
    Encoder_2.runSpeed(STOP_RPM);
  }
}

void cmdBackward(bool backward) {
  if (backward) { //GO
    Encoder_1.runSpeed(-DRIVE_RPM);
    Encoder_2.runSpeed(-DRIVE_RPM);
  } else {      //STOP
    Encoder_1.runSpeed(STOP_RPM);
    Encoder_2.runSpeed(STOP_RPM);
  }
}

void cmdTurnLeft(bool left) {
  if (left) { //GO
    Encoder_1.runSpeed(DRIVE_RPM/5);
    Encoder_2.runSpeed(-DRIVE_RPM/5);
  } else {      //STOP
    Encoder_1.runSpeed(STOP_RPM);
    Encoder_2.runSpeed(STOP_RPM);
  }
}

void cmdTurnRight(bool right) {
  if (right) { //GO
    Encoder_1.runSpeed(-DRIVE_RPM/5);
    Encoder_2.runSpeed(DRIVE_RPM/5);
  } else {      //STOP
    Encoder_1.runSpeed(STOP_RPM);
    Encoder_2.runSpeed(STOP_RPM);
  }
}

void cmdRaiseCrane(bool raiseCrane){
  if (raiseCrane){ //GO
      Encoder_3.runSpeed(DRIVE_RPM/10);
  } else { //STOP
      Encoder_3.runSpeed(STOP_RPM);
  }
}

void cmdLowerCrane(bool lowerCrane){
  if (lowerCrane){ //GO
      Encoder_3.runSpeed(-DRIVE_RPM/10);
  } else { //STOP
      Encoder_3.runSpeed(STOP_RPM);
  }
}

void sendData() {
  JsonObject& root = jsonBufferOut.createObject();

  root["motors"] = jsonBufferOut.createObject();
  JsonArray& drive = jsonBufferOut.createArray();
  drive.add(Encoder_1.getCurPos());  //left
  drive.add(Encoder_2.getCurPos());  //right
  root["motors"]["drive"] = drive;
  root["motors"]["arm"] = Encoder_3.getCurPos(); //arm actuation data
  //  root["motors"]["claw"] = //claw actuation data;

  JsonArray & gyro = jsonBufferOut.createArray();
  Gyro.update();
  gyro.add(Gyro.getAngleX());
  gyro.add(Gyro.getAngleY());
  gyro.add(Gyro.getAngleZ());
  root["gyro"] = gyro;

  JsonArray& line = jsonBufferOut.createArray();
  int lineState = LineFinder.readSensors();
  int mask = 0x01;
  bool leftLine = lineState & mask;
  bool rightLine = (lineState >> 1) & mask;
  line.add(leftLine); //left
  line.add(rightLine); //right
  root["line"] = line;

  root["ultrasonic"] = UltraSensor.distanceCm();
}

void _delay(float seconds) {
  long endTime = millis() + seconds * 1000;
  while (millis() < endTime)_loop();
}

void _loop() {
  Encoder_1.loop();
  Encoder_2.loop();
}

//ultrasonic sensor
float returnDistance() {
  float temp =  UltraSensor.distanceCm();
  delay(100);
  return temp;
}

//linefollower
bool* lineData() {
  int sensorState = LineFinder.readSensors();
  bool toReturn[2];
  switch (sensorState)
  {
    case S1_IN_S2_IN:
      toReturn[0] = true;
      toReturn[1] = true;
      return toReturn; break;
    case S1_IN_S2_OUT:
      toReturn[0] = true;
      toReturn[1] = false;
      return toReturn; break;
    case S1_OUT_S2_IN:
      toReturn[0] = false;
      toReturn[1] = true;
      return toReturn; break;
    case S1_OUT_S2_OUT:
      toReturn[0] = false;
      toReturn[1] = false;
      return toReturn; break;
    default: break;
  }
  delay(200);
}

