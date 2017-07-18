#include <MeMegaPi.h>
#include <Wire.h>
#include <SoftwareSerial.h>



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
  // float start_angle = ...;
  float rotations = distance / DIST_RATIO;
  float move_angle = rotations * 360.0;
  Encoder_1.move(move_angle);
  Encoder_2.move(move_angle);
  // GET END ANGLE FROM ENCODER
  // float start_angle = ...;
  // float start_angle = ...;
}


// ADD SOME MORE MATH
// WILL CONTINUE ROTATING IF OVERSHOOT
//returns differences
float turn(float turn_angle) {
  // GET CURRENT ANGLE FROM GYRO
  // float start_angle = ...;
  // DO MATH TO FIND DESTINATION
  // float dest_angle = curr_angle ...;
  // float curr_angle = start_angle;
  while( /*abs(start_angle-curr_angle) >*/ 5 ) {
    Encoder_1.move(ANGLE_INCR);
    Encoder_2.move(-ANGLE_INCR);
    // UPDATE CURRENT
    // float curr_angle =...;
  }
  return 0.0/*start_angle-curr_angle*/;
}

void setup(){
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

