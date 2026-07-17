#include "alg_nmea.h"

inline fp32 nmea_convert(fp32 raw_degrees)
{
  int firstdigits = ((int)raw_degrees) / 100;
  fp32 nexttwodigits = raw_degrees - (fp32)(firstdigits * 100.0f);
  return (fp32)(firstdigits + nexttwodigits / 60.0f);
}