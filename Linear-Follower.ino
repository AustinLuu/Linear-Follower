#include <PID_v1.h>
#include <HCSR04.h>
#include <Servo.h>
UltraSonicDistanceSensor distanceSensor(12, 11);
Servo servo;
//Define Variables we'll be connecting to
double Setpoint, distance, Output, Input; 
double Kp=2, Ki=0.2, Kd=0.2;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
int angle = 90;
void setup()
{
  Serial.begin(9600);
  servo.attach(10);
  servo.write(angle);
  //initialize the variables we're linked to
  Input = distanceSensor.measureDistanceCm();
  Setpoint = 10.0;
  myPID.SetMode(AUTOMATIC); 
  myPID.SetTunings(Kp,Ki,Kd);
  myPID.SetOutputLimits(-45,45);
}

void loop()
{
  Input = distanceSensor.measureDistanceCm();
  myPID.Compute();
  angle = angle + round(30.0*Output/45.0);
  if (angle > 180){
    angle = 180;
  }
  if(angle < 45){
    angle = 45;
  }
  servo.write(angle);
  if (Input > 15){
    Serial.print("15");
  }
  else{
    Serial.print(Input);
  }
  Serial.print(" ");
  Serial.println(Setpoint);

  delay(100);
}
