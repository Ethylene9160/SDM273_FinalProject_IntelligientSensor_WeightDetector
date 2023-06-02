//#pragma once
#ifndef MY_KINEMATICS_DETECTOR
#define MY_KINEMATICS_DETECTOR 3
#include<Arduino.h>
#include <Wire.h>
#include <Math.h>

#ifndef MY_DEMICAL
#define MY_DEMICAL
typedef float demical;
#endif

#include "Kalman.h"
#include "DataInterface.h"
#include "ToWeightData.h"
#include "MyAverageFilter.h"
//typedef int int16_t;//remenber to delete this!
extern int cp_num;

class DataStorager {
  private:
    demical* realVals;
    demical fNorm;
  public:
    DataStorager(demical* realVals, demical fNorm);

    demical* getVals();

    demical getNorm();

    ~DataStorager();
};
class MyKinematicDetector: public AngleDetector, public AverageInteface
{
  private:
    const demical fRad2Deg = 57.295779513f; //rad to degree
    static const int MPU = 0x68; //MPU-6050 I2C addressַ
    static const int nValCnt = 7; //the number of the register once read
    static const demical filterCoeffecient = 0.15;//coeffecient of 1 order filtering
    MyAverageFilter* myFilter;//delete this

    const int nCalibTimes = 1000; //clear time
    int calibData[nValCnt]; //clear data
    int filterPower[10] = {0.02, 0.03, 0.05, 0.05, 0.05, 0.1, 0.1, 0.1, 0.2, 0.3}; //the power of the 10th order filter

    unsigned long nLastTime = 0; //last reading time
    demical fLastRoll = 0.0f; //last Roll
    demical fLastPitch = 0.0f; //last pitch
    Kalman kalmanRoll; //Roll filter
    Kalman kalmanPitch; //Pitch filter

    demical finalPitch;//remenber to detele this
    demical q[4];


    /*
      ��MPU6050д��һ���ֽڵ�����
      ָ���Ĵ�����ַ��һ���ֽڵ�ֵ
    */
    void writeMPUReg(int nReg, unsigned char nVal);

    /* ��MPU6050�������ٶȼ������������¶Ⱥ��������ٶȼ�
      ������ָ����������
    */
    void readAccGyr(int* pVals);

    /*�Դ�����������ͳ�ƣ�У׼ƽ��ƫ����*/
    void calibration();

    /*��MPU6050����һ���ֽڵ�����
      ָ���Ĵ�����ַ�����ض�����ֵ
    */
    unsigned char readMPUReg(int nReg);

    /*get Roll angle*/
    demical getRoll(demical* pRealVals, demical fNorm);

    /*get Pitch angle*/
    demical getPitch(demical* pRealVals, demical fNorm);

    /*get Yaw angle*/
    demical getYaw(demical* pRealVals, demical fNorm);

    /*Te-establish data we have got*/
    void rectify(int* pReadout, demical* pRealVals);

    // test mahony estimator
    void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);

    // test mahony estimator
    demical q2pitch();

    DataStorager* beforeRead();
    
  public:

    /*
      a init function for initializing this class, with MPU8265 inside.
      Do not use it.
    */
    void begin();

    /*
      constructor.
      Ples initialize it in <code>void setup()</code> function,
      use keyword <code>new</code> to establish this in the stack space,
      will be a better choice.
    */
    MyKinematicDetector();

    void mainLoop();

    demical getRoll();

    demical getPitch();

    demical getYaw();

    virtual demical getAngle();

    virtual void show(mydata data);

    ~MyKinematicDetector();
};


#endif
