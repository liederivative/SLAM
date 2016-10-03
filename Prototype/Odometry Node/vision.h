// Header for Library Pixy Camera
// University of Wolverhampton
// Author: Albert Jimenez
// Date: 30/06/2016

#define X_CENTER 160L
#define Y_CENTER 100L
#define RCS_MIN_POS 0L
#define RCS_MAX_POS 1000L
#define RCS_CENTER_POS ((RCS_MAX_POS-RCS_MIN_POS)/2)
#include <SPI.h>
#include <Pixy.h>

struct blocks_sense {
  double d,signature,angle;
  int s; //size
};
class Vision
{
public:
  Vision(){};
  Pixy pixy;
  blocks_sense data[];
  bool state;
  double *h;
  void init();
  void sense();
};
