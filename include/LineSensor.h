#ifndef LINESENSOR_H__
#define LINESENSOR_H__

#include "API.h"

typedef struct LineSensor {
  int toggle_level;
  unsigned char port;
} LineSensor;

LineSensor lineSensorCreate(unsigned char port, int toggle);

int lineSensorHasLine(const LineSensor* lineSensor);

#endif
