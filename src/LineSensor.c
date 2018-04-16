#include "LineSensor.h"

#include "API.h"
#include "log.h"
#include "util.h"

#include <math.h>

LineSensor lineSensorCreate(unsigned char port, int toggle)
{
  return (LineSensor) {.toggle_level = toggle, .port = port};
}

int lineSensorHasLine(const LineSensor* lineSensor)
{
  int val = analogRead(lineSensor->port);
  printf("Line = %d\n", val);
  return (int)(val < lineSensor->toggle_level);
}
