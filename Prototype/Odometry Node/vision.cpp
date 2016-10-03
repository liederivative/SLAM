// Library for Pixy Camera
// University of Wolverhampton
// Author: Albert Jimenez
// Date: 30/06/2016

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif
#include "math.h"
#include "vision.h"

void Vision::init(){
  pixy.init();
  state = false;
  h = 0;
}
void Vision::sense(){

  static int i = 0;
  unsigned long j;
  uint16_t blocks;
  int count = 0;
  this->state = false;
  blocks = pixy.getBlocks();
  if (blocks)
  {
      i++;

      if (i%1==0)
      {
          float f = 2.8; // focal lenght
          float real_height = 80; // millimetres
          int img_h = 200; // pixels
          int img_w = 320; // pixels
          float sensor_height = 2.43; // millimetres
          float sensor_width = 3.888; // millimetres
          float signature;
          double x = 0;

          for (j=0; j<blocks; j++)
            {

                x = (float) pixy.blocks[j].x;

                double height = pixy.blocks[j].height;
                double width = pixy.blocks[j].width;

                signature = pixy.blocks[j].signature;

                float dx = (X_CENTER - x)*(sensor_width/img_w); // delta x
                double d = (double)(f*real_height*img_h)/(height*sensor_height); // distance to object
                double wr = (d*dx)/f; // component for anglule measurement
                double angle = atan(wr/d); // measured angle

                double ratio_measurement = height/width;

                if ( (ratio_measurement >= 1.9) && (ratio_measurement <= 2.23) ){
                  this->state = true;
                  count++;

                  Serial.print(d);
                  Serial.print("|");
                  Serial.print(angle,6);

                  Serial.print("|");
                  Serial.print(signature); Serial.print("||");
                  // varial for tests
                  this->data[j].signature = signature;
                  this->data[j].d = d;
                  this->data[j].angle = angle;
                }

            }
        }
        this->data[0].s = count;
      }


}
