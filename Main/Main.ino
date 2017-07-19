#include <MeMegaPi.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include "MeGyro.h"
#include "MeUltrasonicSensor.h"
#include "MeLineFollower.h"


//Encoder Motor
MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);

//Math stuff
double angle_rad = PI/180.0;
double angle_deg = 180.0/PI;

// Very important
float ENC_MOTOR_GEAR_RATIO = 46.67;
float REG_MOTOR_GEAR_RATIO = 75;
float DIST_RATIO = 1.2;   //CHANGE THIS NUMBER
int ANGLE_INCR = 5;       //CHANGE THIS NUMBER

void isr_process_encoder1(void)
{
      if(digitalRead(Encoder_1.getPortB()) == 0){
            Encoder_1.pulsePosMinus();
      }else{
            Encoder_1.pulsePosPlus();
      }
}

void isr_process_encoder2(void)
{
      if(digitalRead(Encoder_2.getPortB()) == 0){
            Encoder_2.pulsePosMinus();
      }else{
            Encoder_2.pulsePosPlus();
      }
}

void move(int direction, int speed)
{
      int leftSpeed = 0;
      int rightSpeed = 0;
      if(direction == 1){
            leftSpeed = -speed;
            rightSpeed = speed;
      }else if(direction == 2){
            leftSpeed = speed;
            rightSpeed = -speed;
      }else if(direction == 3){
            leftSpeed = speed;
            rightSpeed = speed;
      }else if(direction == 4){
            leftSpeed = -speed;
            rightSpeed = -speed;
      }
      Encoder_1.setTarPWM(rightSpeed);
      Encoder_2.setTarPWM(leftSpeed);
}
void moveDegrees(int direction,long degrees, int speed_temp)
{
      speed_temp = abs(speed_temp);
      if(direction == 1)
      {
            Encoder_1.move(degrees,(float)speed_temp);
            Encoder_2.move(-degrees,(float)speed_temp);
      }
      else if(direction == 2)
      {
            Encoder_1.move(-degrees,(float)speed_temp);
            Encoder_2.move(degrees,(float)speed_temp);
      }
      else if(direction == 3)
      {
            Encoder_1.move(degrees,(float)speed_temp);
            Encoder_2.move(degrees,(float)speed_temp);
      }
      else if(direction == 4)
      {
            Encoder_1.move(-degrees,(float)speed_temp);
            Encoder_2.move(-degrees,(float)speed_temp);
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
  return start_angle-Encoder_1.getCurPos();
}


// ADD SOME MORE MATH
// WILL CONTINUE ROTATING IF OVERSHOOT
//returns differences
float turn(float turn_angle) {
  int fine_angle
  // GET CURRENT ANGLE FROM GYRO
  Gyro.update();
  double start_angle = Gyro.getAngleZ();
  // DO MATH TO FIND DESTINATION
  float dest_angle = (start_angle + turn_angle) % 360;
  double curr_angle = start_angle;
  while( abs(dest_angle-curr_angle) > ANGLE_INCR ) {
    Encoder_1.move(ANGLE_INCR);
    Encoder_2.move(-ANGLE_INCR);
    // UPDATE CURRENT ANGLE
    Gyro.update();
    curr_angle = Gyro.getAngleZ();
  }
  return (start_angle-curr_angle);
}

void setup(){
    Gyro.begin();
    attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
    attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
    Encoder_1.setPulse(8);
    Encoder_2.setPulse(8);
    Encoder_1.setRatio(REG_MOTOR_GEAR_RATIO);     //86 RPM motor
    Encoder_2.setRatio(ENC_MOTOR_GEAR_RATIO);  //186 RPM motor
    Encoder_1.setPosPid(1.8,0,1.2);
    Encoder_2.setPosPid(1.8,0,1.2);
    Encoder_1.setSpeedPid(0.18,0,0);
    Encoder_2.setSpeedPid(0.18,0,0);
}

void loop(){
    _loop();
}

void _delay(float seconds){
    long endTime = millis() + seconds * 1000;
    while(millis() < endTime)_loop();
}

void _loop(){
    Encoder_1.loop();
    Encoder_2.loop();
}

//gyro
MeGyro::MeGyro(uint8_t port) : MePort(port)
{
  Device_Address = GYRO_DEFAULT_ADDRESS;
}

void MeGyro::setpin(uint8_t AD0, uint8_t INT)
{
  _AD0 = AD0;
  _INT = INT;
#ifdef ME_PORT_DEFINED
  s1 = AD0;
  s2 = INT;
#endif // ME_PORT_DEFINED
}

void MeGyro::begin(void)
{
  gSensitivity = 65.5; //for 500 deg/s, check data sheet
  gx = 0;
  gy = 0;
  gz = 0;
  gyrX = 0;
  gyrY = 0;
  gyrZ = 0;
  accX = 0;
  accY = 0;
  accZ = 0;
  gyrXoffs = 0;
  gyrYoffs = 0;
  gyrZoffs = 0;
  Wire.begin();
  delay(800);
  writeReg(0x6b, 0x00);//close the sleep mode
  writeReg(0x1a, 0x01);//configurate the digital low pass filter
  writeReg(0x1b, 0x08);//set the gyro scale to 500 deg/s

  deviceCalibration();
}

void MeGyro::update(void)
{
  static unsigned long	last_time = 0;
  int8_t return_value;
  double dt, filter_coefficient;
  /* read imu data */
  return_value = readData(0x3b, i2cData, 14);
  if(return_value != 0)
  {
    return;
  }

  double ax, ay, az;
  /* assemble 16 bit sensor data */
  accX = ( (i2cData[0] << 8) | i2cData[1] );
  accY = ( (i2cData[2] << 8) | i2cData[3] );
  accZ = ( (i2cData[4] << 8) | i2cData[5] );
  gyrX = ( ( (i2cData[8] << 8) | i2cData[9] ) - gyrXoffs) / gSensitivity;
  gyrY = ( ( (i2cData[10] << 8) | i2cData[11] ) - gyrYoffs) / gSensitivity;
  gyrZ = ( ( (i2cData[12] << 8) | i2cData[13] ) - gyrZoffs) / gSensitivity;
  ax = atan2(accX, sqrt( pow(accY, 2) + pow(accZ, 2) ) ) * 180 / 3.1415926;
  ay = atan2(accY, sqrt( pow(accX, 2) + pow(accZ, 2) ) ) * 180 / 3.1415926;

  dt = (double)(millis() - last_time) / 1000;
  last_time = millis();

  if(accZ > 0)
  {
    gx = gx - gyrY * dt;
    gy = gy + gyrX * dt;
  }
  else
  {
    gx = gx + gyrY * dt;
    gy = gy - gyrX * dt;
  }
  gz += gyrZ * dt;
  gz = gz - 360 * floor(gz / 360);
  if(gz > 180)
  {
    gz = gz - 360;
  }

  /*
     complementary filter
     set 0.5sec = tau = dt * A / (1 - A)
     so A = tau / (tau + dt)
  */
  filter_coefficient = 0.5 / (0.5 + dt);
  gx = gx * filter_coefficient + ax * (1 - filter_coefficient);
  gy = gy * filter_coefficient + ay * (1 - filter_coefficient);
}

uint8_t MeGyro::getDevAddr(void)
{
  return Device_Address;
}

double MeGyro::getAngleX(void)
{
  return gx;
}

double MeGyro::getAngleZ(void)
{
  return gz;
}

double MeGyro::getGyroX(void)
{
  return gyrX;
}

double MeGyro::getGyroY(void)
{
  return gyrY;
}

double MeGyro::getAngle(uint8_t index)
{
  if(index == 1)
  {
    return getAngleX();
  }
  else if(index == 2)
  {
    return getAngleY();
  }
  else if(index == 3)
  {
    return getAngleZ();
  }
}

void MeGyro::deviceCalibration(void)
{
  int8_t return_value;
  uint16_t x = 0;
  uint16_t num = 500;
  long xSum	= 0, ySum = 0, zSum = 0;
  for(x = 0; x < num; x++)
  {
    return_value = readData(0x43, i2cData, 6);
    xSum += ( (i2cData[0] << 8) | i2cData[1] );
    ySum += ( (i2cData[2] << 8) | i2cData[3] );
    zSum += ( (i2cData[4] << 8) | i2cData[5] );
  }
  gyrXoffs = xSum / num;
  gyrYoffs = ySum / num;
  gyrZoffs = zSum / num;
}

int8_t MeGyro::writeReg(int16_t reg, uint8_t data)
{
  int8_t return_value = 0;
  return_value = writeData(reg, &data, 1);
  return(return_value);
}

int8_t MeGyro::readData(uint8_t start, uint8_t *buffer, uint8_t size)
{
  int16_t i = 0;
  int8_t return_value = 0;
  Wire.beginTransmission(Device_Address);
  return_value = Wire.write(start);
  if(return_value != 1)
  {
    return(I2C_ERROR);
  }
  return_value = Wire.endTransmission(false);
  if(return_value != 0)
  {
    return(return_value);
  }
  delayMicroseconds(1);
  /* Third parameter is true: relase I2C-bus after data is read. */
  Wire.requestFrom(Device_Address, size, (uint8_t)true);
  while(Wire.available() && i < size)
  {
    buffer[i++] = Wire.read();
  }
  delayMicroseconds(1);
  if(i != size)
  {
    return(I2C_ERROR);
  }
  return(0); //return: no error
}

int8_t MeGyro::writeData(uint8_t start, const uint8_t *pData, uint8_t size)
{
  int8_t return_value = 0;
  Wire.beginTransmission(Device_Address);
  return_value = Wire.write(start);
  if(return_value != 1)
  {
    return(I2C_ERROR);
  }
  Wire.write(pData, size);
  return_value = Wire.endTransmission(true);
  return(return_value); //return: no error
} // end of gyro code

//ultrasonic sensor

MeUltrasonicSensor::MeUltrasonicSensor(void) : MePort(0)
{

}

void MeUltrasonicSensor::setpin(uint8_t SignalPin)
{
  _SignalPin = SignalPin;
  _lastEnterTime = millis();
  _measureFlag = true;
  _measureValue = 0;
#ifdef ME_PORT_DEFINED
  s2 = _SignalPin;
#endif // ME_PORT_DEFINED
}

double MeUltrasonicSensor::distanceCm(uint16_t MAXcm)
{
  long distance = measure(MAXcm * 55 + 200);
  if(distance == 0)
  {
    distance = MAXcm * 58;
  }
  return( (double)distance / 58.0);
}

double MeUltrasonicSensor::distanceInch(uint16_t MAXinch)
{
  long distance = measure(MAXinch * 145 + 200);
  if(distance == 0)
  {
    distance = MAXinch * 148;
  }
  return( (double)(distance / 148.0) );
}


long MeUltrasonicSensor::measure(unsigned long timeout)
{
  long duration;
  if(millis() - _lastEnterTime > 23)
  {
    _measureFlag = true;
  }

  if(_measureFlag == true)
  {
    _lastEnterTime = millis();
    _measureFlag = false;
    MePort::dWrite2(LOW);
    delayMicroseconds(2);
    MePort::dWrite2(HIGH);
    delayMicroseconds(10);
    MePort::dWrite2(LOW);
    pinMode(s2, INPUT);
    duration = pulseIn(s2, HIGH, timeout);
    _measureValue = duration;
  }
  else
  {
    duration = _measureValue;
  }
  return(duration);
} //end of ultrasonic sensor


//line follower code

MeLineFollower::MeLineFollower(void) : MePort(0)
{

}

void MeLineFollower::setpin(uint8_t Sensor1,uint8_t Sensor2)
{
  _Sensor1 = Sensor1;
  _Sensor2 = Sensor2;
  pinMode(_Sensor1,INPUT);
  pinMode(_Sensor2,INPUT);
#ifdef ME_PORT_DEFINED
  s1 = _Sensor1;
  s2 = _Sensor2;
#endif // ME_PORT_DEFINED
}

uint8_t MeLineFollower::readSensors(void)
{
  uint8_t state	= S1_IN_S2_IN;
#ifdef ME_PORT_DEFINED
  bool s1State = MePort::dRead1();
  bool s2State = MePort::dRead2();
#else // ME_PORT_DEFINED
  bool s1State = digitalRead(_Sensor1);
  bool s2State = digitalRead(_Sensor2);
#endif // ME_PORT_DEFINED
  state = ( (1 & s1State) << 1) | s2State;
  return(state);
}

bool MeLineFollower::readSensor1(void)
{
#ifdef ME_PORT_DEFINED
  return(MePort::dRead1() );
#else // ME_PORT_DEFINED
  return digitalRead(_Sensor1);
#endif // ME_PORT_DEFINED
}

bool MeLineFollower::readSensor2(void)
{
#ifdef ME_PORT_DEFINED
	return(MePort::dRead2() );
#else // ME_PORT_DEFINED
  return digitalRead(_Sensor2);
#endif // ME_PORT_DEFINED
}
