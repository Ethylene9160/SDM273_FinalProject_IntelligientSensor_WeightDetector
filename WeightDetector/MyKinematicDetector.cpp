#include "MyKinematicDetector.h"
//#define invSqrt sqrtf


/**********Long Average Filter********/

#ifndef MY_DATA
#define MY_DATA
typedef int mydata;
#endif
extern int cp_num;

demical invSqrt(demical x);
void deleteAll(ListNode* node);
/**********class MyKinematicDetector*********/
void MyKinematicDetector::begin()
{
  //Serial.begin(9600);
  Wire.begin();
  writeMPUReg(0x6B, 0);

  calibration();
  nLastTime = micros(); //record current time
}

void MyKinematicDetector::writeMPUReg(int nReg, unsigned char nVal)
{
  Wire.beginTransmission(MPU);
  Wire.write(nReg);
  Wire.write(nVal);
  Wire.endTransmission(true);
  /*
    Wire.beginTransmission(0x68); //开启MPU-6050的传输
    Wire.write(0x1C); //加速度倍率寄存器的地址
    Wire.requestFrom(0x68, 1, true); //先读出原配置
    unsigned char acc_conf = Wire.read();
    int f = 3;
    acc_conf = ((acc_conf & 0xE7) | (f << 3));
    Wire.write(acc_conf);
    Wire.endTransmission(true); //结束传输，true表示释放总线
  */
}

unsigned char MyKinematicDetector::readMPUReg(int nReg)
{
  Wire.beginTransmission(MPU);
  Wire.write(nReg);
  Wire.requestFrom(MPU, 1, true);
  Wire.endTransmission(true);
  return Wire.read();
}

void MyKinematicDetector::readAccGyr(int* pVals)
{
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.requestFrom(MPU, nValCnt * 2, true);
//  unsigned char acc_conf = Wire.read();
//  int f = 3;
//  acc_conf = ((acc_conf & 0xE7) | (f << 3));
//  Wire.write(acc_conf);
  Wire.endTransmission(true);
  for (long i = 0; i < nValCnt; ++i) {
    pVals[i] = Wire.read() << 8 | Wire.read();
  }
}

void MyKinematicDetector::calibration()
{
  demical valSums[7] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0 };
  //?????
  for (int i = 0; i < nCalibTimes; ++i) {
    int mpuVals[nValCnt];
    readAccGyr(mpuVals);
    for (int j = 0; j < nValCnt; ++j) {
      valSums[j] += mpuVals[j];
    }
  }
  //???????
  for (int i = 0; i < nValCnt; ++i) {
    calibData[i] = int(valSums[i] / nCalibTimes);
  }
  calibData[2] += 16384; //??о?Z????????￡??趨?????????
}

demical MyKinematicDetector::getRoll(demical* pRealVals, demical fNorm)
{
  demical fNormXZ = sqrt(pRealVals[0] * pRealVals[0] + pRealVals[2] * pRealVals[2]);
  demical fCos = fNormXZ / fNorm;
  return acos(fCos) * fRad2Deg;
}

demical MyKinematicDetector::getYaw(demical* pRealVals, demical fNorm)
{
  demical fNormXY = sqrt(pRealVals[0] * pRealVals[0] + pRealVals[1] * pRealVals[1]);
  demical fCos = fNormXY / fNorm;
  return acos(fCos) * fRad2Deg;
}

demical MyKinematicDetector ::getPitch(demical* pRealVals, demical fNorm) {
  demical fNormYZ = sqrt(pRealVals[1] * pRealVals[1] + pRealVals[2] * pRealVals[2]);
  demical fCos = fNormYZ / fNorm;
  return acos(fCos) * fRad2Deg;
}

void MyKinematicDetector::rectify(int* pReadout, demical* pRealVals)
{
  for (int i = 0; i < 3; ++i) {
    pRealVals[i] = (demical)(pReadout[i] - calibData[i]) / 16384.0f;
  }
  pRealVals[3] = pReadout[3] / 340.0f + 36.53;
  for (int i = 4; i < 7; ++i) {
    pRealVals[i] = (demical)(pReadout[i] - calibData[i]) / 131.0f;
  }
}



MyKinematicDetector::MyKinematicDetector()
{

  //this->startNode = new ListNode(0, l8);
  //(*this).begin();
  //this->filterPower = {0.3,0.2,0.1,0.1,0.1,0.05,0.05,0.05,0.03,0.02};
  //this->myFilter = new MyAverageFilter(this, 10, 10, 0.05);
  cp_num += 2;

  //test mahony estimator
  q[0] = 1.0f;
  Serial.print("q0 init:");
  Serial.println(q[0]);
  q[1] = 0.0f;
  q[2] = 0.0f;
  q[3] = 0.0f;
}

void MyKinematicDetector::mainLoop()
{
  int readouts[nValCnt];
  readAccGyr(readouts);

  demical realVals[7];
  rectify(readouts, realVals);


  demical fNorm = sqrt(realVals[0] * realVals[0] + realVals[1] * realVals[1] + realVals[2] * realVals[2]);
  //Serial.print("fNorm: ");
  //Serial.println(fNorm);
  demical fRoll = getRoll(realVals, fNorm);
  if (realVals[1] > 0) {
    fRoll = -fRoll;
  }
  demical fPitch = getPitch(realVals, fNorm);
  if (realVals[0] < 0) {
    fPitch = -fPitch;
  }

  unsigned long nCurTime = micros();
  demical dt = (double)(nCurTime - nLastTime) / 1000000.0;

  demical fNewRoll = kalmanRoll.getAngle(fRoll, realVals[4], dt);
  demical fNewPitch = kalmanPitch.getAngle(fPitch, realVals[5], dt);

  demical fRollRate = (fNewRoll - fLastRoll) / dt;
  demical fPitchRate = (fNewPitch - fLastPitch) / dt;

  fLastRoll = fNewRoll;
  fLastPitch = fNewPitch;
  nLastTime = nCurTime;

  demical coef = 32768.f / 2000.f / fRad2Deg;
  MahonyAHRSupdateIMU((demical)realVals[5]*coef, (demical)realVals[4]*coef, (demical)realVals[6]*coef, realVals[1], realVals[0], realVals[2]);
  /*for(int i = 0; i < 4; ++i){
    Serial.print("q");
    Serial.print(i);
    Serial.print("is: ");
    Serial.println(*(q+i));
    }*/
  demical q2p = q2pitch();

  Serial.print("real4:");
  Serial.println(q2p);
  /*
    Serial.print("Roll:");
    Serial.print(fNewRoll);
    //Serial.print('(');
    //Serial.print(fRollRate);
    Serial.print("\nPitch:");
    Serial.print(fNewPitch);
    //Serial.print('(');
    //Serial.print(fPitchRate);
    Serial.print("\n");
  */
  //delay(10);
}

demical MyKinematicDetector::q2pitch()
{
  return asinf(2.f * (q[0] * q[2] - q[1] * q[3]));
}

DataStorager *MyKinematicDetector::beforeRead()
{
  int readouts[nValCnt];
  readAccGyr(readouts);

  demical realVals[7];
  rectify(readouts, realVals);
  demical fNorm = sqrt(realVals[0] * realVals[0] + realVals[1] * realVals[1] + realVals[2] * realVals[2]);
  cp_num++;
  return new DataStorager(realVals, fNorm);
}

demical MyKinematicDetector::getRoll()
{
  DataStorager* storager = beforeRead();
  demical rv = *(storager->getVals());
  demical fNorm = storager->getNorm();
  cp_num--;
  delete storager;
  return this->getRoll(&rv, fNorm);
}

demical MyKinematicDetector::getPitch()
{
  /*
    DataStorager* storager = beforeRead();

    demical fPitch = this->getPitch(storager->getVals(), storager->getNorm());

    if (*(storager->getVals()) < 0) {
    fPitch = -fPitch;
    }
    unsigned long nCurTime = micros();
    demical dt = (double)(nCurTime - nLastTime) / 1000000.0;

    //demical fNewPitch = kalmanPitch.getAngle(fPitch, storager->getVals()[5], dt);
    //demical fNewPitch = fPitch*filterCoeffecient + (1.f-filterCoeffecient)*fLastPitch;
    push(fPitch);
    pop();

    demical fNewPitch = calculateOutput();
    demical fPitchRate = (fNewPitch - fLastPitch) / dt;
    myFilter->update(ToWeightData::analog2digit(fNewPitch, 0));
    //demical fPitchRate = (fNewPitch - fLastPitch) / dt;

    fLastPitch = fNewPitch;
    nLastTime = nCurTime;
    cp_num--;
    delete storager;
    return fNewPitch;
  */



  int readouts[nValCnt];
  readAccGyr(readouts);

  demical realVals[7];
  rectify(readouts, realVals);


  demical fNorm = sqrt(realVals[0] * realVals[0] + realVals[1] * realVals[1] + realVals[2] * realVals[2]);

  demical fPitch = getPitch(realVals, fNorm);
  if (realVals[0] < 0) {
    fPitch = -fPitch;
  }
  unsigned long nCurTime = micros();
  demical dt = (double)(nCurTime - nLastTime) / 1000000.0;

  demical fNewPitch = fPitch * filterCoeffecient + (1.f - filterCoeffecient) * fLastPitch;
  //myFilter->update(ToWeightData::analog2digit(fNewPitch, 0));
  demical fPitchRate = (fNewPitch - fLastPitch) / dt;
  fLastPitch = fNewPitch;
  nLastTime = nCurTime;
  return fNewPitch;

}

demical MyKinematicDetector::getYaw()
{
  /*
    DataStorager* storager = beforeRead();
    demical rv = *(storager->getVals());
    demical fNorm = storager->getNorm();
    cp_num--;
    delete storager;


    return this->getYaw(&rv, fNorm);
  */
  int readouts[nValCnt];
  readAccGyr(readouts);

  demical realVals[7];
  rectify(readouts, realVals);


  demical fNorm = sqrt(realVals[0] * realVals[0] + realVals[1] * realVals[1] + realVals[2] * realVals[2]);

  demical fPitch = getYaw(realVals, fNorm);
  if (realVals[0] < 0) {
    fPitch = -fPitch;
  }
  unsigned long nCurTime = micros();
  demical dt = (double)(nCurTime - nLastTime) / 1000000.0;
  //push(fPitch);
  //pop();
  //demical fNewPitch = calculateOutput();

  demical fNewPitch = fPitch * filterCoeffecient + (1.f - filterCoeffecient) * fLastPitch;

  /*
    push(fPitch);
    pop();

    demical fNewPitch = calculateOutput();
  */
  //myFilter->update(ToWeightData::analog2digit(fNewPitch, 0));
  demical fPitchRate = (fNewPitch - fLastPitch) / dt;
  fLastPitch = fNewPitch;
  nLastTime = nCurTime;
  return fNewPitch;
}


demical MyKinematicDetector::getAngle() {
  return this->getPitch();
  //return this->getYaw();
}

//test mahony estimator
volatile float twoKp = 1.0f;                      // 2 * proportional gain (Kp)
volatile float twoKi = 0.0f;                      // 2 * integral gain (Ki)
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f; // integral error terms scaled by Ki

void MyKinematicDetector::MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;
  float sampleFreq = 1000.f;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  // 只在加速度计数据有效时才进行运算
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    // 将加速度计得到的实际重力加速度向量v单位化
    recipNorm = invSqrt(ax * ax + ay * ay + az * az); //该函数返回平方根的倒数

    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;


    // Estimated direction of gravity
    // 通过四元数得到理论重力加速度向量g
    // 注意，这里实际上是矩阵第三列*1/2，在开头对Kp Ki的宏定义均为2*增益
    // 这样处理目的是减少乘法运算量
    halfvx = q[1] * q[3] - q[0] * q[2];
    halfvy = q[0] * q[1] + q[2] * q[3];
    halfvz = q[0] * q[0] - 0.5f + q[3] * q[3];

    // Error is sum of cross product between estimated and measured direction of gravity
    // 对实际重力加速度向量v与理论重力加速度向量g做外积
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Compute and apply integral feedback if enabled
    // 在PI补偿器中积分项使能情况下计算并应用积分项
    if (twoKi > 0.0f) {
      // integral error scaled by Ki
      // 积分过程
      integralFBx += twoKi * halfex * (1.0f / sampleFreq);
      integralFBy += twoKi * halfey * (1.0f / sampleFreq);
      integralFBz += twoKi * halfez * (1.0f / sampleFreq);

      // apply integral feedback
      // 应用误差补偿中的积分项
      gx += integralFBx;
      gy += integralFBy;
      gz += integralFBz;
    }
    else {
      // prevent integral windup
      // 避免为负值的Ki时积分异常饱和
      integralFBx = 0.0f;
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    // 应用误差补偿中的比例项
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }

  // Integrate rate of change of quaternion
  // 微分方程迭代求解
  gx *= (0.5f * (1.0f / sampleFreq));   // pre-multiply common factors
  gy *= (0.5f * (1.0f / sampleFreq));
  gz *= (0.5f * (1.0f / sampleFreq));
  qa = q[0];
  qb = q[1];
  qc = q[2];
  q[0] += (-qb * gx - qc * gy - q[3] * gz);
  q[1] += (qa * gx + qc * gz - q[3] * gy);
  q[2] += (qa * gy - qb * gz + q[3] * gx);
  q[3] += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  // 单位化四元数 保证四元数在迭代过程中保持单位性质
  recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  q[0] *= recipNorm;
  q[1] *= recipNorm;
  q[2] *= recipNorm;
  q[3] *= recipNorm;

  //Mahony官方程序到此结束，使用时只需在函数外进行四元数反解欧拉角即可完成全部姿态解算过程
}


void MyKinematicDetector::show(mydata d) {
  //Serial.print("myAverageFilter: ");
  //Serial.println(d);
}

MyKinematicDetector::~MyKinematicDetector() {
  cp_num--;
  //delete this->myFilter;
}
/********** end MyKinematic class ********/

/********* class MyKinematic class *******/
DataStorager::DataStorager(demical* realVals, demical fNorm)
{
  this->realVals = new demical[7];
  cp_num++;
  for (int i = 0; i < 7; i++) {
    this->realVals[i] = realVals[i];
  }
  this->fNorm = fNorm;
}

demical* DataStorager::getVals()
{
  return this->realVals;
}

demical DataStorager::getNorm()
{
  return this->fNorm;
}

DataStorager:: ~DataStorager() {
  cp_num--;
  delete[] this->realVals;
}
/*********** end DataStorager class *********/

/********** struct ListNode *************/
/*
ListNode::ListNode(): ListNode(0) {}
ListNode::ListNode(demical val): ListNode(0, nullptr) {}
ListNode::ListNode(demical x, ListNode* node) {
  val = x;
  next = node;
}
*/
/*********** end ListNode ***************/
void deleteAll(ListNode* node) {
  while (node != nullptr) {
    ListNode* n = node;
    node = node->next;
    delete n;
    cp_num--;
  }
}


/******other function****/
demical invSqrt(demical x)
{
  //Serial.print("invertSqrt:");
  //Serial.println(x);
  demical xhalf = 0.5f * x;
  long i = *(long*)&x;          // get bits for floating value
  i =  0x5f375a86 - (i >> 1);  // gives initial guess
  x = *(demical*)&i;            // convert bits back to float
  x = x * (1.5f - xhalf * x * x); // Newton step
  return x;
  //return 1.f/sqrtf((x));
}
