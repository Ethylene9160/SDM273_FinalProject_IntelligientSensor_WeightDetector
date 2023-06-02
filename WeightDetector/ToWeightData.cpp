#include "ToWeightData.h"
#include <arduino.h>

mydata ToWeightData::analog2digit(demical data, demical deltax) {
  data = data - deltax;

  mydata output;
  if (data >= -5.29 && data < 0) {
    output = (int)((((-1.67 * data - 19.15) * data - 76.27) * data - 134.7) * data - 121.8) * 10;
  }
  else if (data >= -36.17 && data < -5.29) {
    output = (int)((((-2.159e-05 * data - 0.001674) * data - 0.05108) * data - 0.7145) * data - 14.59) * 10;
  }
  else if (data >= -40 && data < -36.17) {
    output = (int)(((-1.065 * data - 201.7) * data - 1.528e+04) * data - 5.789e+05) * 10;
  }
  else {
    // Handle data outside of defined ranges
    output = 0;
  }

  int tmp = output % 10;
  if (tmp > 5) {
    output = output / 10 + 1;
  }
  else {
    output = output / 10;
  }





  return output;
}

mydata ToWeightData::analog2error(demical data) {
  return (int)(500.f * data);
}

mydata ToWeightData::data2weight(mydata data) {
  return 0;//todo
}

mydata ToWeightData::digit2analog(demical data) {
  return mydata(data);
}


mydata ToWeightData::smallTransport(mydata data) {
  Serial.print("small data ");
  float in = -((float)data / 500.f);
  float tmp = 0;
  mydata out = 0;
  /*if (in > 30) {
    Serial.println("lager: ");
    //    tmp = (((-0.0497 * in +7.1)*in-376.78)*in+8829.9)-76933;

    tmp = (2.17 * in - 119) * in + 1966.5;
  }*/
  
  if(in>29.0310){
    tmp = (0.6522*in-21.978)*in+411.93;
  }
  //  else if (in > 18.6298) {
  else if (in > 20.757) {

    tmp = 13.076 * in - 56.049;

    //    tmp = 12.4481*in-37.2614;
  } else {
    Serial.println("lower: ");
    //tmp = ((0.0058*in - 0.1732) * in + 10.989) * in +11.686 ;
    //tmp = 9.5362 * in + 14.856;

    tmp = 9.743 * in + 13.134;
  }
  //if(tmp < 18) return 0;
  out = mydata(tmp * 10.f);
  if (out % 10 > 5) out = out / 10 + 1;
  else out = out / 10;
  return out;
}

mydata ToWeightData::largeTransport(mydata data) {
  Serial.println("large data: ");
  float in = -((float)data / 500.f);
  float tmp = 0;
  mydata out = 0;
  //tmp = (((-0.0247*in+2.658)*in-105.22)*in+1844.3)*in-11702.f;
  //tmp = ((0.1209*in-8.2956)*in+212.84)*in-1495.9;
  //tmp = (0.744 * in - 8.3201) * in + 290.06;
  //tmp = ((0.0387*in-2.1113)*in+59.704)*in-243.59;
  tmp = (0.8378 * in - 14.307) * in + 366.77;
  out = mydata(tmp * 10.f);
  if (out % 10 > 5) out = out / 10 + 1;
  else out = out / 10;
  return out;
}
