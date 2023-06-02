//#include<arduino.h>
//#include "Big_Num/big_num.h"
#ifndef MY_DEMICAL
#define MY_DEMICAL
typedef float demical;
#endif

#ifndef MY_DATA
#define MY_DATA
typedef int mydata;
#endif
unsigned char tubeNums[] = {  //common positive
  B11000000, // 0
  B11111001, // 1
  B10100100, // 2
  B10110000, // 3
  B10011001, // 3
  B10010010, // 8
  B10000010, // 6
  B11111000, // 7
  B10000000, // 8
  B10010000 // 9
};
int cp_num = 0;
const int LOAD = 5;
const int SCLK = 6;
const int SDI = 7;

const int LATCH_PIN = LOAD;
const int CLOCK_PIN = SCLK;
const int DATA_PIN = SDI;

const int ZERO_KEY = 11;
const int SMALL_KEY = 10;
const int LARGE_KEY = 9;

bool isLarge = false;
bool isYiChu = false;
#include"Kalman.h"
#include"MyAverageFilter.h"
#include"MyKinematicDetector.h"
#include"ToWeightData.h"
#include"DataInterface.h"
#include"MyDigitalTube.h"

mydata difference;
int calculator = 1;

void light(int i) {
  if (i > 9999)i = 9999;
  else if (i < 0) i = 0;
  unsigned char gewei = (i % 100) % 10;
  unsigned char shiwei = (i % 100) / 10;
  unsigned char baiwei = (i % 1000) / 100;
  unsigned char qianwei = i / 1000;
  digitalWrite(LATCH_PIN, LOW); //低电位表示启动
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, tubeNums[gewei]);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, tubeNums[shiwei] & 0b01111111);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, tubeNums[baiwei]);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, tubeNums[qianwei]);
  digitalWrite(LATCH_PIN, HIGH); //高电位表示停止
  delayMicroseconds(2);
  digitalWrite(LATCH_PIN, HIGH);//ST_CP
  //delay(1000);

}

class Stablizer: public AverageInteface {
  private:
    MyDigitalTube* digitalTube;
    void begin();
  public:
    MyAverageFilter* filter;
    Stablizer(MyDigitalTube* digitalTube);
    Stablizer(int length, MyDigitalTube* digitalTube);
    Stablizer(int length, int absoluteError, int relativeError, MyDigitalTube* digitalTube);

    void update(mydata d);
    virtual void show(mydata data) override;
};

MyKinematicDetector* kinematicDetector;
MyDigitalTube* myTube = nullptr;
Stablizer* stablizer;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  kinematicDetector = new MyKinematicDetector();
  myTube = new MyDigitalTube(LATCH_PIN, CLOCK_PIN, DATA_PIN);
  stablizer = new Stablizer(4, 25, 3, myTube);
  cp_num += 3;
  kinematicDetector->begin();
  Serial.println("initialization of KinematicDetector is okey.");
  //stablizer->begin();
  //myTube->begin();
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);
  pinMode(SMALL_KEY, INPUT);
  pinMode(LARGE_KEY, INPUT);
  pinMode(ZERO_KEY, INPUT);
  Serial.println("set pinmode fine");
  //while(!Serial);
  /*
    while (!stablizer->filter->isBalence) {
    for (int i = 0; i < 200; i++) {
      stablizer->update(ToWeightData::analog2error(kinematicDetector->getAngle()));
    }
    }
  */
  //difference = (demical)stablizer->filter->getAverage()/(500.f);
  //stablizer->filter->reset(3,5);
  //Serial.println("OK");
}

void loop() {
  //    Serial.print("the number of current instances is: ");
  //    Serial.println(cp_num);
  //light(2333);
  //myTube->light(2333);
  //delay(1000);
  //return to 0 point

  if (digitalRead(ZERO_KEY) == HIGH) {
    delay(5);
    if (digitalRead(ZERO_KEY) == HIGH) {
      Serial.println("I'm button!! why press me !!");
      difference = /*(float)*/stablizer->filter->getAverage();
    }
  }

  if (digitalRead(SMALL_KEY) == HIGH) {
    delay(5);
    if (digitalRead(SMALL_KEY) == HIGH) {
      Serial.println("I'm SMALL_BUTTON!! why press me !!");
      isLarge = false;
    }
  }

  if (digitalRead(LARGE_KEY) == HIGH) {
    delay(5);
    if (digitalRead(LARGE_KEY) == HIGH) {
      Serial.println("I'm LARGE_BUTTON!! why press me !!");
      isLarge = true;
    }
  }

  demical d = kinematicDetector->getAngle();
  stablizer->update(ToWeightData::analog2error(d));
//  Serial.print("before filter:");
//  Serial.println(ToWeightData::analog2error(d-difference));
  delay(2);
}



Stablizer::Stablizer(MyDigitalTube* digitalTube): digitalTube(digitalTube) {
  filter = new MyAverageFilter(this);
  cp_num++;
  begin();
}
Stablizer::Stablizer(int length, MyDigitalTube* digitalTube): digitalTube(digitalTube) {
  filter = new MyAverageFilter(this, length);
  cp_num++;
  begin();
}
Stablizer::Stablizer(int length, int absoluteError, int relativeError, MyDigitalTube* digitalTube): digitalTube(digitalTube) {
  filter = new MyAverageFilter(this, length, absoluteError, relativeError);
  cp_num++;
  begin();
}
//override this method, and after you push a data in this stablizer, this method will be used.
void Stablizer::show(mydata data)
{
  if(--calculator) return;
  
  calculator = 10;
  mydata out=0;
  
  Serial.print(" Filted:");
  
  if (isLarge) {
    out = ToWeightData::largeTransport((data - difference));
  } else {
    out = ToWeightData::smallTransport((data - difference));
  }
  
//  out = ((data - difference));
  
  
  

  Serial.print(out);
  Serial.println();
  //    this->digitalTube->light((int)data);
  light(out);
  //if(data > 9999) data = 9999;
  //if(data < 0) data = 0;
  //if(myTube != nullptr)myTube->light(data);
  
}

void Stablizer::update(mydata d) {
  filter->update(d);
}

void Stablizer::begin() {
  //myTube->begin();
  this->digitalTube->begin();
}
