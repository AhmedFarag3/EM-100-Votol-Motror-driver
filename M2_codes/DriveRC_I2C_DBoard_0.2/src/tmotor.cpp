/*#include <tmotor.h>

unsigned int float_to_uint(float x, float x_min, float x_max, int bit)
{ /// converts a float to an unsigned int, given range and number of bits ///
  //Serial.println("conv2");
  //Serial.println(x);
  //delay(20);
  float span = x_max - x_min;
  float offset = x_min;
  //Serial.println(offset);
  unsigned int pgg = 0;
  if (bit == 12)
  {
    pgg = (x - offset) * 4095 / span;
  }
  if (bit == 16)
  {
    pgg = (x - offset) * 65535 / span;
  }
  return pgg;
}
*/