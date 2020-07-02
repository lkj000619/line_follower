#include <SoftwareSerial.h>
#include <StepperMulti.h>

SoftwareSerial BTSerial(2, 3);

StepperMulti stepper_L(2048, 4, 6, 5, 7);
StepperMulti stepper_R(2048, 8, 10, 9, 11);

#define IR_L A0
#define IR_R A1

void BT_data(int s1, int s2, int m1, int m2);

void setup() {
  BTSerial.begin(9600);
  Serial.begin(9600);
  Serial.println("ATcommand");  //ATcommand Start
  Serial.println("CLEARDATA");

  Serial.println("LABEL,left_s, right_s, left_m, right_m"); //라벨 작성

  pinMode(IR_L, INPUT);
  pinMode(IR_R, INPUT);
}
int loop_time = 0;
int stepper_speed = 14;
const int target_line_value = 80;

double error_I = 0.0;
double error_L = 0.0;
double error_R = 0.0;
double errorPrev_L = 0.0;
double errorPrev_R = 0.0;
double error_D = 0.0;

float Kp = 0.02;
float Kd = 0.07l;
float Ki = 0.0;

long lastTime = 0;
int set_speed1 = 14;
int set_speed2 = 14;

void loop() {
  long currTime = millis();

  int value_L = analogRead(IR_L);
  int value_R = analogRead(IR_R);

  value_L = constrain(value_L + random(-20, 20), 0, 999);
  value_R = constrain(value_R + random(-20, 20), 0 , 999);

  //PID control
  int average = (value_L + value_R) / 2;
  int data1 = 900 / value_L;
  int data2 = value_R / 48 + 3;
  int gain_L = 0;
  int gain_R = 0;

  if (value_L > 700 && value_R > 700) {
    stepper_L.setStep(0);
    stepper_R.setStep(0);
  }

  else {
    int R_tarnfor = 0;
    int L_tarnfor = 0;

    if (value_L < 200 && value_R < 150) {
      stepper_L.setSpeed(14);
      stepper_R.setSpeed(14);
    }
    //if (data1 - data2 == 0) {
    if (value_L > 400) {
      error_L = value_L - 50;
      error_I += error_L * (currTime - lastTime);
      error_D = (error_L - errorPrev_L) / (currTime - lastTime);

      int gain_L = (Kp * error_L + Ki * error_I + Kd * error_D);
      int L_tarnfor = map(gain_L, 5 , 16 , 0 , 14);
      //Serial.print("L-tranfor \t");
      //Serial.print(gain_L);
      //Serial.print("\t");
      set_speed2 = 14 - L_tarnfor;
      set_speed2 = constrain((set_speed2 + random(-1, 3)), -14, 14);
      //Serial.print(set_speed2);
      //Serial.print("\t");
    }
    if (value_R > 300) {
      error_R = value_R - 50;
      error_I += error_R * (currTime - lastTime);
      error_D = (error_R - errorPrev_R) / (currTime - lastTime);

      int gain_R = (Kp * error_R + Ki * error_I + Kd * error_D);
      int R_tarnfor = map(gain_R, 4 , 15 , 0 , 14);
      //Serial.print("R-tranfor \t");
      //Serial.print(R_tarnfor);
      //Serial.print("\t");
      set_speed1 = 14 - R_tarnfor;
      set_speed1 = constrain((set_speed1 + random(-1, 2)), -14 , 14);
    }


    stepper_L.setSpeed(set_speed1);
    stepper_R.setSpeed(set_speed2);
    //Serial.print(set_speed1);
    //Serial.print("\t");
    //Serial.print(set_speed2);
    //Serial.print("\t");


    //}
    stepper_L.setStep(-2048);
    stepper_R.setStep(2048);

    lastTime = currTime;
    errorPrev_L = error_L;
    errorPrev_R = error_R;
  }

  
  //BT_data(value_R, value_L, set_speed1, set_speed2);
  /*
  loop_time += 1;
  if (loop_time == 5) {
    stepper_L.moveStep();
    stepper_R.moveStep();
    loop_time = 0 ;
  }
  */

}

void BT_data(int sss1, int sss2, int mmmm1, int mmmm2) {
  BTSerial.print("DATA,");
  BTSerial.print(0);
  BTSerial.print(",");
  BTSerial.print(sss1);
  BTSerial.print(",");
  BTSerial.print(sss2);
  BTSerial.print(",");
  BTSerial.print(mmmm1);
  BTSerial.print(",");
  BTSerial.println(mmmm2);
}
