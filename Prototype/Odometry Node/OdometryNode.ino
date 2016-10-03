 // Odometry Node
 // University of Wolverhampton
 // Author: Albert Jimenez
 // Date: 30/06/2016

#include <Arduino.h>
#include <math.h>
#include "vision.h"

int L1 =6, L2 = 7;
int L3 = 9; //ENA
int L4 =8; //ENB
// L1 and L3 must be connected to PWM pins


/// Robot measurements
const double R = 27.5; //millimetres
const double L = 138; //millimetres
const int Encoder_N = 18; // thicks
////////////////////////////////////////////////////////
//////////////// Odometry Model ///////////////////////

 const int DL0 = 3, DL1 = 5, DR0 = 2, DR1 = 4;//7; // Pins for Right and Left encoders. DL0 and DR0 must be Interrupts
 int lStat0, lStat1, rStat0, rStat1;
 volatile int lpos = 0, rpos = 0;  // variables keeping count of the pulses on each side


void intLeft();
void intRight();
void get_v_o(double,double,double *);
double pi_to_pi(double);
void get_wr_wl(double,double,double *);

class Encoders{

  double old_tick_l,old_tick_r,new_tick_r,new_tick_l;
  public:
  Encoders(){};
  void set_old_tick(double r, double l);
  void set_new_tick(double r,double l);
  double delta_tick_l();
  double delta_tick_r();
};
double Encoders::delta_tick_r(){
  return new_tick_r - old_tick_r;
}
double Encoders::delta_tick_l(){
  return new_tick_l - old_tick_l;
}
void Encoders::set_old_tick(double r, double l){
  old_tick_l = l;
  old_tick_r = r;
}
void Encoders::set_new_tick(double r, double l){
  new_tick_l = l;
  new_tick_r = r;
}

//variables sample
double sampletime = 100;
unsigned long ref; // reference variable for timesamples

// Vision Pixy
Vision visionSensor ;
// Encoders
Encoders encoder;
double old_tick_r,old_tick_l = 0;
unsigned long lastTime;
double Wl,Wr;
void setup()
{
  // Motor Driver Controller
  // initialize the digital pins we will use as an output.
  pinMode(L1, OUTPUT);
  pinMode(L2, OUTPUT);
  pinMode(L3, OUTPUT);
  pinMode(L4, OUTPUT);
  Serial.begin(115200);


  // Encoder inputs
  // initialise input pins and interrupts for speed sensors
  pinMode(DL0, INPUT);
  pinMode(DL1, INPUT);
  pinMode(DR0, INPUT);
  pinMode(DR1, INPUT);
  attachInterrupt(digitalPinToInterrupt(DL0), intLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(DR0), intRight, RISING);

  encoder.set_old_tick(0, 0);



  // Vision Sensor Init
  visionSensor.init();
  // encoder sample
  ref = millis();
  lastTime = millis();
}

// odometry structure
struct odo{
  double x = 0.0;
  double y=0.0;
  double theta=0.0;
  void update(double vl,double vr){
    double Dc = (vr + vl)/2.0;
    double angle = (vr - vl)/L ;
    this->x += Dc * cos(this->theta);
    this->y += Dc * sin(this->theta);
    this->theta += angle;
    this->theta = pi_to_pi(theta);
  }
};
odo metric;
int count =0;
bool flag = false;
String data = "0";
int movements[][2] = {{0,0},{127,127},{10,240},{240,10}}; // fix speed for movements
int index = 0;

void loop()
{

    // select option for input
      switch (data.toInt()) {
        case 8:
          Serial.println("Move forward");
          index = 1;
          // forward(500, 255,255);
          break;
        case 4:
          Serial.println("Turn left");
          index = 2;
          // leftTurnFd(300, 80, 255);
          break;
        case 6:
          Serial.println("Turn right");
          index = 3;
          // rightTurnFd(300, 255, 80);
          break;
        default:
            halt(500);

       }
       data = "0"; // Clear data

      unsigned long timenow = millis();
      ////////// Sample of position ////////
      if (timenow- ref >=sampletime) {

          unsigned long diff = timenow- ref;
          encoder.set_new_tick((double) rpos, (double) lpos);


          double diff_tick_r = rpos - old_tick_r;
          double diff_tick_l = lpos - old_tick_l;
          old_tick_r = (double) rpos;
          old_tick_l = (double) lpos;
          encoder.set_old_tick((double) rpos, (double) lpos);

          double DR = 2*PI*R*(diff_tick_r/Encoder_N); //distance right wheel
          double DL = 2*PI*R*(diff_tick_l/Encoder_N); //distance left wheel



          double v_omega[2] = {0,0};
          get_v_o((DL/R),(DR/R),v_omega);
          Serial.print("control: ");Serial.print(v_omega[0]);
          Serial.print("|");Serial.print(v_omega[1]);Serial.print(",");
          metric.update(DL, DR);

          Serial.print("Odometry: "); Serial.print(metric.x);Serial.print("|");Serial.print(metric.y);Serial.print("|");Serial.print(metric.theta,4);


          ref = millis();

          count +=1;
          Serial.print(",distance: ");
          visionSensor.sense();
          Serial.print("0.0");
          Serial.print("|");Serial.print("0.0");Serial.print("|");Serial.println("0.0");
          Serial.print("data: ");Serial.println(visionSensor.data[1].d);
          Serial.print("size: ");Serial.println(sizeof(visionSensor.data[0].s));

      }

      unsigned long current = millis();
      if (current - lastTime >= 100){
        forward(sampletime,movements[index][0] ,movements[index][1]);
        index = 0;
      }

  ////////////////////////////////////////////////////////////////////////////////////////


  ///////////////////////////////////////////////////////////////////////////////////////


}
void serialEvent() {
  // rotune for serial inputs
  while (Serial.available()) {

    Serial.println("--------------------*****");
    char inChar = (char)Serial.read();
    data = inChar;
    Serial.println(inChar);
    if (inChar == '\n') {
      flag = true; //set flag
    }
  }
}

// Interrupts routines for encoders
void intLeft()  // Interrupt on right side encoder (DL0)
{

  lStat0 = digitalRead(DL0);
  lStat1 = digitalRead(DL1);

  if(lStat1 > 0)
    lpos++;
  else
    lpos--;

}

void intRight()    // Interrupt on right side encoder (DR0)
{
  rStat0 = digitalRead(DR0);
  rStat1 = digitalRead(DR1);

    if( rStat1 > 0 )
      rpos--;
    else
      rpos++;

}

// robMove -- routines to the L298N for the direction selected.
void robMove(int l1, int l2, int r1, int r2)
{
  analogWrite(L1, l1);
  digitalWrite(L2, l2);
  analogWrite(L3, r1);
  digitalWrite(L4, r2);
}



void forward(int wait, int vSpeedl, int vSpeedr)
{
  bool cond_l, cond_r =  LOW;

  if (vSpeedl <0){
    vSpeedl = 255-abs(vSpeedl);
    cond_l = HIGH;
  }
  if (vSpeedr <0){
    vSpeedr = 255-abs(vSpeedr);
    cond_r = HIGH;
  }

  robMove(vSpeedl, cond_l, vSpeedr, cond_r);
  delay(wait);
}




void halt(int wait)
{
  // Serial.println("Stopping");
  robMove(0, LOW, 0, LOW);
  delay(wait);
}

void get_v_o(double vl__, double vr__, double *array){
    // v_l = left-wheel angular velocity (rad/s)
    // v_r = right-wheel angular velocity (rad/s)

    array[0] = ( R / 2.0 ) * ( vl__ + vr__ );
    array[1] = ( R / L ) * ( vr__ - vl__ );

}
double pi_to_pi(double ang){
  return atan2(sin(ang),cos(ang));
}
void get_wr_wl(double v, double o, double *res){
  res[0] = ( (2.0 * v) - (o*L) ) / (2.0 * R); // in rad/s
  res[1] = ( (2.0 * v) + (o*L) ) / (2.0 * R);
}
