#include "MyDigitalTube.h"
#include <arduino.h>
//#include <iostream>
MyDigitalTube::MyDigitalTube(int latchPin, int colckPin, int dataPin)
{
  this->latchPin = latchPin;
  this->clockPin = clockPin;
  this->dataPin = dataPin;
  //this->begin();
}

void MyDigitalTube::light(int i)
{
  unsigned char gewei = (i % 100) % 10;
  unsigned char shiwei = (i % 100) / 10;
  unsigned char baiwei = (i % 1000) / 100;
  unsigned char qianwei = i / 1000;

  digitalWrite(latchPin, LOW); //low voltage to start
  shiftOut(dataPin, clockPin, MSBFIRST, tubeNums[gewei]);
  //delayMicroseconds(2);

  shiftOut(dataPin, clockPin, MSBFIRST, tubeNums[shiwei]);
  //delayMicroseconds(2);
  shiftOut(dataPin, clockPin, MSBFIRST, tubeNums[baiwei]);
  //delayMicroseconds(2);
  shiftOut(dataPin, clockPin, MSBFIRST, tubeNums[qianwei]);
  digitalWrite(latchPin, HIGH); //high voltage to over
  delayMicroseconds(2);
  //delay(1);
  digitalWrite(latchPin, HIGH);//ST_CP

  //std::cout << gewei << " " << shiwei << " " << baiwei << " " << qianwei << std::endl;
}

void MyDigitalTube::begin()
{
  //pinMode(latchPin, OUTPUT);
  //pinMode(clockPin, OUTPUT);
  //pinMode(dataPin, OUTPUT);
}
