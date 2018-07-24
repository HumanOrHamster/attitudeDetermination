#include "Arduino.h"
#include<Wire.h>
#include<math.h>


// Description
/*
By Binh B.
07/23/2018 first commit

*/


#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

#define PI 3.1415926
#define twoPI 6.2831853
#define PItwo 1.570796

// LSM303DLHC ACC and MAG Addresses
const int LSM303_ACC_ADDR 			= 0b0011001;
const int LSM303_MAG_ADDR 			= 0b0011110;

// MAG Register LSM303DLHC
const int CRA_REG_M         = 0x00;
const int CRB_REG_M         = 0x01;
const int MR_REG_M          = 0x02;
const int OUT_X_H_M 		    = 0x03; // starting read register for mag


// tbd on usage
struct sensorParams
{
    float xSF;
    float ySF;
    float zSF;
} lsm303Mag;

// ACC Register LSM303DLHC
const int CTRL_REG4_A       = 0x23;
const int CTRL_REG1_A       = 0x20;
const int OUT_X_L_A 		    = 0x28; // starting read register for acc

// L3GD20 Address
const int L3GD20_GYRO_ADDR	= 0b1101011;

// GYRO Register L3GD20
const int CTRL_REG1_G 		= 0x20;
const int CTRL_REG4_G 		= 0x23;
const int OUT_X_L_G 		= 0x28;

const float r2d = 57.29577951308232;

///////////
// GLOBAL PARAMS
int mRaw[3] = {0, 0, 0};
int aRaw[3] = {0, 0, 0};
int gRaw[3]= {0,0,0};

// gRaw corrected for offset
int gyroDigiCorr[3] = {0,0,0};

float mGauss[3] = {0.0, 0.0, 0.0};
float aGrav[3] = {0.0, 0.0, 0.0};
float gRad[3] = {0.0, 0.0, 0.0};
float aNorm[3] = {0.0, 0.0, 0.0};

int mMin[3] = { -665, -749, -577};
int mMax[3] = {505, 404, 442};
int mOffset[3] = {0, 0, 0};

// gyro bias
int gyroRawBias[3] = {-333,61,-237};

// accel bias
int accelRawBias[3] = {-1113,-42,-1620};

// accel external g protection bound (10% of 16384 digi) is error of 1638
int oneGDigit = 16384; // 2g full scale at 16 bit
int extForceBoundDigi = 0.1*oneGDigit;
int aMagDigi = 0;

// mag external protection bound ~0.47 gauss nominal, +/- 10%
float nomGaussMag = 0.47;
float nomGaussBound = 0.15*nomGaussMag;
bool isMagNomValid = true;



float RefRpy[3] = {0.0, 0.0, 0.0};
float SolRpy[3] = {0.0,0.0,0.0}; // fused solution
float magGauss = 0.0;
float headingDeg = 0.0;

//gyro rate in rad
float gyroRateRad[3] = {0.0, 0.0, 0.0}; // rad/sec

// QUATERNION STATES
float quatRatePrev[4] = {0.0, 0.0, 0.0, 0.0};
float quatPrev[4] = {1.0, 0.0, 0.0, 0.0};
float quatSol[4] = {1.0, 0.0, 0.0, 0.0};

// rpy filtering
float Krpy[3]= {0.05, 0.05, 0.01}; // proportional gains

// execution dt
float stepSize = 0.02;
unsigned long stepSizeMicroSec = stepSize*1000000;

// timer
unsigned long t0;
unsigned long t1;
unsigned long t2;
unsigned long t3;


void printDataToSerial(byte opt);
void printVec(int (&vec)[3]);
void setupLsm();
void setupL3G();
void readRawLsm(bool applyAccelOffset, int (&mRawData)[3],int (&aBiasData)[3], int (&aRawData)[3]);
void readRawL3G(bool applyOffset, int (&gBiasData)[3],int (&gRawData)[3]);
void getGdigiMagnitude(int (&aRawData)[3], int& aMag );
void rawGyroToRad(int (&gRawData)[3], float (&rpyAngle)[3]);
void calGyroBias(int (&gRawData)[3], int (&gBiasData)[3]);
void accelBiasCal(int (&aRawData)[3], int (&aBiasData)[3]);
void calGyroAndAccel(int (&aRawData)[3], int (&aBiasData)[3],int (&gRawData)[3], int (&gBiasData)[3]);
void getRefRPY(int (&mRawData)[3], float (&mGaussData)[3], int (&mOffsetData)[3],bool& isMagValid, int (&aRawData)[3], float (&aNormData)[3], float (&refRpyData)[3]);
void vectorCopy(int (&vecA)[3], float (&vecB)[3]);
void writeToReg(int addr, int reg, int value);
void calMag(int (&mMinData)[3], int (&mMaxData)[3], int (&offsetData)[3]);
void setupMagOffset(int (&mMinData)[3], int (&mMaxData)[3], int (&offsetData)[3]);
void magCalAux(int& minMax, int rawVal, int (&states)[3]);
void integrateAndFilterSoln(float dt, int aMag, bool isMagValid, float (&gyroRate)[3],float (&refRpyData)[3], float (&qRatePrev)[4], float (&qPrev)[4],float (&qSol)[4]);
void quatRateFromBodyRate(float (&gyroRate)[3], float (&q)[4],float (&qDot)[4] );
void quatNorm(float (&q)[4]);
void quat2Euler(float (&q)[4], float (&gyroRpyData)[3]);
void euler2Quat(float (&rpy)[3], float (&q)[4]);
void integrateQuat(float dt, float (&qRate)[4], float (&qRatePrev)[4], float (&qPrev)[4],float (&qOut)[4]);
void rpyFiltering(bool isMagValid, float (&refRpyData)[3],float (&q)[4] );
void wrapAngles(float &angle, int opt);
void printEulerAngles(float (&gyroRpyData)[3]);
void printQuat( float (&q)[4]);
void MatrixVectorMul(float (&matA)[3][3],float (&vecB)[3],float (&vecOut)[3]);
void VectorDotProduct(float (&vecA)[3],float (&vecB)[3],float& dotSol);
void VectorCross(float (&vecA)[3], float (&vecB)[3], float (&vecC)[3]);
void VectorMag(float (&vecA)[3], float& magOut);
void VectorNorm(float (&vecA)[3]);

void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);  // Open serial port
    Wire.begin(); // I2C Setup

    // default is 100 kHz, with 400 kHz you get 4x the speed*/
    Wire.setClock(400000); // set I2C SCL to High Speed Mode of 400kHz
    setupLsm();
    setupL3G();

    /*
    // not used yet but for future implementation, also cant be initialized outside of setup.
    lsm303Mag.xSF = 1100.0;
    lsm303Mag.ySF = 1100.0;
    lsm303Mag.zSF = 980.0;
    */

    // optional mag cal
    //calMag(mMin, mMax, mOffset);


    setupMagOffset(mMin, mMax, mOffset);
    /*
    Serial.print("Offset");
    printVec(mOffset);
    */

    //calGyroBias(gRaw,gyroRawBias);
    //accelBiasCal(aRaw,accelRawBias);
    calGyroAndAccel(aRaw,accelRawBias,gRaw,gyroRawBias);

    readRawLsm(true,mRaw, accelRawBias, aRaw);
    getRefRPY(mRaw, mGauss, mOffset, isMagNomValid, aRaw, aNorm, RefRpy);

    // initialize SolRpy to the refer rpy
    for (int i = 0; i<3; i++)
    {
        SolRpy[i] = RefRpy[i];
    }

    euler2Quat(SolRpy,quatSol);
    euler2Quat(SolRpy,quatPrev);

    Serial.println("Setup Complete");


}

void loop()
{
    // total from t0 to t2 ~ 5.2 ms
    t0 = micros();

    readRawLsm(true,mRaw, accelRawBias, aRaw);
    getRefRPY(mRaw, mGauss, mOffset, isMagNomValid, aRaw, aNorm, RefRpy);

    /*
    if (isMagNomValid == false)
    {
    	Serial.println("mag Nom Value Exceeded");
    }
    */


    readRawL3G(true,gyroRawBias,gRaw);
    rawGyroToRad(gRaw,gyroRateRad);

    getGdigiMagnitude(aRaw,aMagDigi);

    integrateAndFilterSoln(stepSize,aMagDigi,isMagNomValid, gyroRateRad,RefRpy,quatRatePrev, quatPrev,quatSol);

    quat2Euler(quatSol, SolRpy);
    t2 = micros();

    //printEulerAngles(SolRpy);

    printDataToSerial(0b00010000);


    //Serial.print((t2-t0)/1000.0,5);
    //Serial.println(" ms");

    while((micros()-t0)<=stepSizeMicroSec)
    {
        t1 = micros();
    };


}

void printDataToSerial(byte opt)
{
    if ((opt & 0b00000001) > 0)
    {
        Serial.print("rawMag,");

        printVec(mRaw);
    }
    if ((opt & 0b00000010) > 0)
    {
        Serial.print("rawAcc,");
        printVec(aRaw);
    }
    if ((opt & 0b00000100) > 0)
    {
        Serial.print("gyroRaw,");
        printVec(gRaw);
    }

    if ((opt & 0b00001000) > 0)
    {
        Serial.print("roll: ");
        Serial.print(ToDeg(RefRpy[0]));
        Serial.print(" ");
        Serial.print("pitch: ");
        Serial.print(ToDeg(RefRpy[1]));
        Serial.print(" ");
        Serial.print("yaw: ");
        Serial.println(ToDeg(RefRpy[2]));
    }
    if ((opt & 0b00010000) > 0)
    {
        Serial.print("!ANG:,");
        Serial.print(ToDeg(SolRpy[0]));
        Serial.print(",");
        Serial.print(ToDeg(SolRpy[1]));
        Serial.print(",");
        Serial.print(ToDeg(SolRpy[2]));
        Serial.println();
    }
}

void printVec(int (&vec)[3])
{
    Serial.print(vec[0]);
    Serial.print(",");
    Serial.print(vec[1]);
    Serial.print(",");
    Serial.println(vec[2]);
}



void setupLsm()
{
    // MAG SETUP
    // 0x10 = 0b00010000
    // DO = 100 (15 Hz ODR)
    writeToReg(LSM303_MAG_ADDR, CRA_REG_M, 0x10);

    // 0x20 = 0b00100000
    // GN = 001 (+/- 1.3 gauss full scale)
    writeToReg(LSM303_MAG_ADDR, CRB_REG_M, 0x20);

    // 0x00 = 0b00000000
    // MD = 00 (continuous-conversion mode)
    writeToReg(LSM303_MAG_ADDR, MR_REG_M, 0x00);

    // ACC Setup
    // 0x08 = 0b00001000
    // FS = 00 (+/- 2 g full scale); HR = 1 (high resolution enable)
    // FS +/- 2g (16 bit)
    // (2^15-1)/2 = 16384 digits/g or 1/16384 = 0.000061035 mg/digit
    writeToReg(LSM303_ACC_ADDR, CTRL_REG4_A, 0x08);

    // 0x57 = 0b01010111
    // ODR = 0101 (100 Hz); Zen = Yen = Xen = 1 (all axes enabled)
    writeToReg(LSM303_ACC_ADDR, CTRL_REG1_A, 0x57);
}

void setupL3G()
{

    // GYRO SETUP

    //0x77 0b01111111
    // DR = 01 (190Hz), BW = 11 (70Hz),PD = 1 (normal mode), Zen=Xen=Yen=1

    writeToReg(L3GD20_GYRO_ADDR,CTRL_REG1_G,0x7F);
    //writeToReg(L3GD20_GYRO_ADDR,CTRL_REG1_G,0x6F);

    // 0x01 = 0b00010000
    // FS = 500 dps
    writeToReg(L3GD20_GYRO_ADDR,CTRL_REG4_G,0x10);


}

void readRawLsm(bool applyAccelOffset, int (&mRawData)[3],int (&aBiasData)[3], int (&aRawData)[3])
{

    Wire.beginTransmission(LSM303_ACC_ADDR);
    // need to add a 1 bit to the MSB (SUPER STUPID)
    Wire.write(OUT_X_L_A | (1 << 7)); //
    Wire.endTransmission();
    Wire.requestFrom(LSM303_ACC_ADDR, 6);


    while (Wire.available() < 6); // wait to get bytes

    // see page 22/42 of LSM303DLHC
    aRawData[0] = Wire.read() | Wire.read() << 8;
    aRawData[1] = Wire.read() | Wire.read() << 8;
    aRawData[2] = Wire.read() | Wire.read() << 8;

    Wire.beginTransmission(LSM303_MAG_ADDR);
    Wire.write(OUT_X_H_M); // Trigger outputs starting with register 3B or 59
    Wire.endTransmission();
    Wire.requestFrom(LSM303_MAG_ADDR, 6);

    while (Wire.available() < 6); // wait to get bytes

    // see page 23/42 of LSM303DLHC
    mRawData[0] = Wire.read() << 8 | Wire.read();
    mRawData[2] = Wire.read() << 8 | Wire.read();
    mRawData[1] = Wire.read() << 8 | Wire.read();

    if (applyAccelOffset == true)
    {
        for (int i = 0; i<3; i++)
        {
            aRawData[i] -= aBiasData[i];
        }
    }

}


void readRawL3G(bool applyOffset, int (&gBiasData)[3],int (&gRawData)[3])
{

    Wire.beginTransmission(L3GD20_GYRO_ADDR);
    // need to add a 1 bit to the MSB (SUPER STUPID)
    Wire.write(OUT_X_L_G | (1 << 7)); //
    Wire.endTransmission();
    Wire.requestFrom(L3GD20_GYRO_ADDR, 6);


    while (Wire.available() < 6); // wait to get bytes

    // see page 22/42 of LSM303DLHC
    gRawData[0] = Wire.read() | Wire.read() << 8;
    gRawData[1] = Wire.read() | Wire.read() << 8;
    gRawData[2] = Wire.read() | Wire.read() << 8;

    if (applyOffset == true)
    {
        for (int i = 0; i<3; i++)
        {
            gRawData[i] -= gBiasData[i];
        }
    }
}

void getGdigiMagnitude(int (&aRawData)[3], int& aMag )
{


    aMag = sqrt((unsigned long)aRawData[0]*aRawData[0]+(unsigned long)aRawData[1]*aRawData[1]+(unsigned long)aRawData[2]*aRawData[2]);

}


void rawGyroToRad(int (&gRawData)[3], float (&rpyAngle)[3])
{

    // FS = 500 dps
    // (2^15-1)/500 = 65.534 digit/(deg/sec) or 1/65.534 = 15.26 (mdeg/sec)/digit
    for (int i = 0; i<3; i++)
    {
        rpyAngle[i] = ToRad(0.01525925*gRawData[i]);
    }
}

void calGyroBias(int (&gRawData)[3], int (&gBiasData)[3])
{
    long biasTemp[3] = {0,0,0};

// initialize bias to zero
    for (int i = 0; i<3; i++)
    {
        gBiasData[i] = 0;
    }

// add data points to perform averaging
    int n = 100; // number of points to average
    for (int i = 1; i<=n; i++)
    {
        // read gyro data
        readRawL3G(false,gBiasData,gRawData);

        for (int j = 0; j <3; j++)
        {
            biasTemp[j]+=gRawData[j];
        }
    }

// average
    for (int i = 0; i<3; i++)
    {
        biasTemp[i] /= n;
        gBiasData[i] = biasTemp[i];
    }
    printVec(gBiasData);
}


void accelBiasCal(int (&aRawData)[3], int (&aBiasData)[3])
{
    long biasTemp[3] = {0,0,0};

    int dummy[3] = {0,0,0};
    // initialize bias to zero
    for (int i = 0; i<3; i++)
    {
        aBiasData[i] = 0;
    }

// add data points to perform averaging
    int n = 100; // number of points to average
    for (int i = 1; i<=n; i++)
    {
        // read accelData
        readRawLsm(false, dummy,dummy,aRawData);

        for (int j = 0; j <3; j++)
        {
            biasTemp[j]+=aRawData[j];
        }

    }

// average
    for (int i = 0; i<3; i++)
    {
        biasTemp[i] /= n;
    }

    aBiasData[0] = biasTemp[0];
    aBiasData[1] = biasTemp[1];
    aBiasData[2] = biasTemp[2] + oneGDigit;

    printVec(aBiasData);


}

void calGyroAndAccel(int (&aRawData)[3], int (&aBiasData)[3],int (&gRawData)[3], int (&gBiasData)[3])
{
    long biasTempGyro[3] = {0,0,0};
    long biasTempAccel[3] = {0,0,0};
    int dummy[3] = {0,0,0};


    // add data points to perform averaging
    int n = 100; // number of points to average
    for (int i = 1; i<=n; i++)
    {
        // get data
        readRawLsm(false, dummy,dummy,aRawData);
        readRawL3G(false,dummy,gRawData);

        for (int j = 0; j <3; j++)
        {
            biasTempAccel[j]+=aRawData[j];
            biasTempGyro[j]+=gRawData[j];
        }
    }

// average
    for (int i = 0; i<3; i++)
    {
        biasTempAccel[i] /= n;
        biasTempGyro[i] /= n;
        gBiasData[i] = biasTempGyro[i];
    }

    aBiasData[0] = biasTempAccel[0];
    aBiasData[1] = biasTempAccel[1];
    aBiasData[2] = biasTempAccel[2] + oneGDigit;

    Serial.println("bias accel");
    printVec(aBiasData);
    Serial.println("bias gyro");
    printVec(gBiasData);
}





void getRefRPY(int (&mRawData)[3], float (&mGaussData)[3], int (&mOffsetData)[3],bool& isMagValid, int (&aRawData)[3], float (&aNormData)[3], float (&refRpyData)[3])
{

    float mGaussMag;
    float magErr;
    // see page 37/42 of LSM303DLHC
    // Need to this to scale values of each axis correctly!

    mGaussData[0] = (mRawData[0] - mOffsetData[0]) / 1100.0; //X
    mGaussData[1] = (mRawData[1] - mOffsetData[1]) / 1100.0; //Y
    mGaussData[2] = (mRawData[2] - mOffsetData[2]) / 980.0; //Z

    ///////////////// CHECK FOR EXTERNAL MAG INTEREFERENCE
    VectorMag(mGaussData,mGaussMag);
    magErr = abs(mGaussMag - nomGaussMag);
    if ( magErr >= nomGaussBound )
    {
        isMagValid = false;
    }
    else
    {
        isMagValid = true;
    }

    //////////////

    // right now no need to actually convert gravity raw data

    //normalize gvector (must do this or else incorrect and img values for pitch and roll)
    vectorCopy(aRawData, aNormData);
    VectorNorm(aNormData);

    // pitch
    refRpyData[1] = asin(aNormData[0]);
    // roll
    refRpyData[0] = atan2(-aNormData[1], -aNormData[2]);
    // yaw/heading

    float mvecX_NED = cos(refRpyData[1])*mGaussData[0] + sin(refRpyData[0])*sin(refRpyData[1])*mGaussData[1]+cos(refRpyData[0])*sin(refRpyData[1])*mGaussData[2];
    float mvecY_NED = cos(refRpyData[0])*mGaussData[1]-sin(refRpyData[0])*mGaussData[2];
    refRpyData[2] = -atan2(mvecY_NED,mvecX_NED);

}


void vectorCopy(int (&vecA)[3], float (&vecB)[3])
{
    // set vec B == vec A
    vecB[0] = (float)vecA[0];
    vecB[1] = (float)vecA[1];
    vecB[2] = (float)vecA[2];

}


void writeToReg(int addr, int reg, int value)
{

    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();

}


void calMag(int (&mMinData)[3], int (&mMaxData)[3], int (&offsetData)[3])
{
    // NOTE DUE TO THE USAGE OR ARDUINO LIBRARY, THIS CANNOT BE MADE A LIBRARY

    int numSamples = 300; // ~ 30 secs
    int xMinState[3] = {0, 0, 0};
    int yMinState[3] = {0, 0, 0};
    int zMinState[3] = {0, 0, 0};
    int xMaxState[3] = {0, 0, 0};
    int yMaxState[3] = {0, 0, 0};
    int zMaxState[3] = {0, 0, 0};

    int mRawData[3] = {0, 0, 0};
    int dummy[3] = {0, 0, 0};

    // initialize using global min max
    for (int i = 0; i < 3; i++)
    {
        xMinState[i] = mMinData[0];
        yMinState[i] = mMinData[1];
        zMinState[i] = mMinData[2];
        xMaxState[i] = mMaxData[0];
        yMaxState[i] = mMaxData[1];
        zMaxState[i] = mMaxData[2];
    }

    // cal mx
    for (int i = 0; i < numSamples; i++)
    {
        readRawLsm(false,mRawData,dummy, dummy);

        if (mRawData[0] > mMaxData[0])
        {
            magCalAux(mMaxData[0], mRawData[0], xMaxState);
        }
        if (mRawData[1] > mMaxData[1])
        {
            magCalAux(mMaxData[1], mRawData[1], yMaxState);
        }
        if (mRawData[2] > mMaxData[2])
        {
            magCalAux(mMaxData[2], mRawData[2], zMaxState);
        }
        if (mRawData[0] < mMinData[0])
        {
            magCalAux(mMinData[0], mRawData[0], xMinState);
        }
        if (mRawData[1] < mMinData[1])
        {
            magCalAux(mMinData[1], mRawData[1], yMinState);
        }
        if (mRawData[2] < mMinData[2])
        {
            magCalAux(mMinData[2], mRawData[2], zMinState);
        }

        // cant have this in the partition code
        Serial.println("");
        Serial.print("Min: ");
        printVec(mMinData);

        Serial.print("Max: ");
        Serial.println("");
        Serial.print("Min: ");

        delay(100);
    }

    setupMagOffset(mMinData, mMaxData, mOffset);
}

void setupMagOffset(int (&mMinData)[3], int (&mMaxData)[3], int (&offsetData)[3])
{
    offsetData[0] = (mMinData[0] + mMaxData[0]) / 2;
    offsetData[1] = (mMinData[1] + mMaxData[1]) / 2;
    offsetData[2] = (mMinData[2] + mMaxData[2]) / 2;
}

void magCalAux(int& minMax, int rawVal, int (&states)[3])
{
    minMax = (rawVal + states[0] + states[1] + states[2]) / 4.0; // floor
    states[2] = states[1];
    states[1] = states[0];
    states[0] = rawVal;
}



void integrateAndFilterSoln(float dt, int aMag, bool isMagValid, float (&gyroRate)[3],float (&refRpyData)[3], float (&qRatePrev)[4], float (&qPrev)[4],float (&qSol)[4])
{
    // Step 1 compute quat rate
    float qRate[4] = {0.0, 0.0, 0.0, 0.0};

    quatRateFromBodyRate(gyroRate,qSol,qRate);

    // Step 2 integrate quat rate
    integrateQuat(dt, qRate, qRatePrev,qPrev,qSol);

    // Step 3 normalize integrate quat
    quatNorm(qSol);

    // Step 4 fuse data
    if (abs(aMag - oneGDigit) <= extForceBoundDigi)
    {
        rpyFiltering(isMagValid,refRpyData,qSol);
    }

    // set state
    for (int i=0; i<4; i++)
    {
        qPrev[i] = qSol[i];
        qRatePrev[i] = qRate[i];
    }
}

void quatRateFromBodyRate(float (&gyroRate)[3], float (&q)[4],float (&qDot)[4] )
{
    // gyroRate, xyz at the body level in rad/sec
    // q is quaternion vecor where the first element is magnitude, and the next 3 elem defines the vector
    qDot[0] = 0.5*(-q[1]*gyroRate[0] - q[2]*gyroRate[1] - q[3]*gyroRate[2]);
    qDot[1] = 0.5*(q[0]*gyroRate[0] + q[2]*gyroRate[2] - q[3]*gyroRate[1]);
    qDot[2] = 0.5*(q[0]*gyroRate[1] - q[1]*gyroRate[2] + q[3]*gyroRate[0]);
    qDot[3] = 0.5*(q[0]*gyroRate[2] + q[1]*gyroRate[1] - q[2]*gyroRate[0]);
}

void quatNorm(float (&q)[4])
{
    float normVal = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    q[0] /= normVal;
    q[1] /= normVal;
    q[2] /= normVal;
    q[3] /= normVal;
}

void quat2Euler(float (&q)[4], float (&gyroRpyData)[3])
{
    float test = q[0]*q[2]-q[3]*q[1];

    if (test > 0.5)
    {
        gyroRpyData[0] = 0.0;
        gyroRpyData[1] = 1.570796326794897;
        gyroRpyData[2] = -2*atan2(q[1],q[0]);
    }
    else if (test < -0.5)
    {
        gyroRpyData[0] = 0.0;
        gyroRpyData[1] = -1.570796326794897;
        gyroRpyData[2] = 2*atan2(q[1],q[0]);
    }
    else
    {
        gyroRpyData[0] = atan2(2*(q[0]*q[1]+q[2]*q[3]),1-2*(q[1]*q[1]+q[2]*q[2]));
        gyroRpyData[1] = asin(2*(q[0]*q[2]-q[3]*q[1]));
        gyroRpyData[2] = atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]));
    }
}


void euler2Quat(float (&rpy)[3], float (&q)[4])
{
    float cosPhi2 = cos(rpy[0]/2.0);
    float cosTheta2 = cos(rpy[1]/2.0);
    float cosPsi2 = cos(rpy[2]/2.0);

    float sinPhi2 = sin(rpy[0]/2.0);
    float sinTheta2 = sin(rpy[1]/2.0);
    float sinPsi2 = sin(rpy[2]/2.0);

    q[0] = cosPhi2*cosTheta2*cosPsi2 + sinPhi2*sinTheta2*sinPsi2;
    q[1] = sinPhi2*cosTheta2*cosPsi2 - cosPhi2*sinTheta2*sinPsi2;
    q[2] = cosPhi2*sinTheta2*cosPsi2 + sinPhi2*cosTheta2*sinPsi2;
    q[3] = cosPhi2*cosTheta2*sinPsi2 - sinPhi2*sinTheta2*cosPsi2;
}

void integrateQuat(float dt, float (&qRate)[4], float (&qRatePrev)[4], float (&qPrev)[4],float (&qOut)[4])
{
    for (int i=0; i<4; i++)
    {
        qOut[i] = qPrev[i] + 0.5*(qRate[i] + qRatePrev[i])*dt;
    }
}


void rpyFiltering(bool isMagValid, float (&refRpyData)[3],float (&q)[4] )
{

    float rpy[3] = {0,0,0};
    float rpyErr[3] = {0,0,0};

    // convert to Euler angles
    quat2Euler(q, rpy);

    // compute errors
    for (int i = 0; i<3; i++)
    {
        rpyErr[i] = refRpyData[i] - rpy[i];
    }

    // add angle wrapping to prevent angle overflow
    wrapAngles(rpyErr[0],1); // +/- pi
    wrapAngles(rpyErr[1],2); // +/- pi/2
    wrapAngles(rpyErr[2],1); // +/- pi

    // apply Gains to get new estimated rpy
    // note Krpy is a global param
    rpy[0] = rpy[0] + Krpy[0]*rpyErr[0];
    rpy[1] = rpy[1] + Krpy[1]*rpyErr[1];

    if (isMagValid == true)
    {
        rpy[2] = rpy[2] + Krpy[2]*rpyErr[2];
    }

    // convert rpy to quat
    euler2Quat(rpy, q);
}

void wrapAngles(float &angle, int opt)
{

    if (opt == 1)
    {
        // +/- pi
        while ( angle > PI )
        {
            angle = angle - twoPI;
        }
        //+/- 2pi
        while ( angle < -PI )
        {
            angle = angle + twoPI;
        }
    }
    else
    {
        while ( angle > PItwo )
        {
            angle = angle - PI;
        }

        while ( angle < -PItwo )
        {
            angle = angle + PI;
        }
    }
}



void printEulerAngles(float (&gyroRpyData)[3])
{
    Serial.print("roll: ");
    Serial.print(ToDeg(gyroRpyData[0]),5);
    Serial.print(" ");
    Serial.print("pitch: ");
    Serial.print(ToDeg(gyroRpyData[1]),5);
    Serial.print(" ");
    Serial.print("yaw: ");
    Serial.println(ToDeg(gyroRpyData[2]),5);
}

void printQuat( float (&q)[4])
{
    Serial.print("q: ");
    for (int i = 0; i < 4; i++)
    {
        Serial.print(q[i],5);
        Serial.print(", ");
    }
    Serial.println(" ");
}


void MatrixVectorMul(float (&matA)[3][3],float (&vecB)[3],float (&vecOut)[3])
{

    vecOut[0] = vecB[0]*matA[0][0]+vecB[1]*matA[0][1]+vecB[2]*matA[0][2];
    vecOut[1] = vecB[0]*matA[1][0]+vecB[1]*matA[1][1]+vecB[2]*matA[1][2];
    vecOut[2] = vecB[0]*matA[2][0]+vecB[1]*matA[2][1]+vecB[2]*matA[2][2];

}

void VectorDotProduct(float (&vecA)[3],float (&vecB)[3],float& dotSol)
{
    dotSol= vecA[0]*vecB[0]+vecA[1]*vecB[1]+vecA[2]*vecB[2];

}

void VectorCross(float (&vecA)[3], float (&vecB)[3], float (&vecC)[3])
{
    vecC[0] = vecA[1]*vecB[2]-vecB[1]*vecA[2];
    vecC[1] = -vecA[0]*vecB[2]+vecB[0]*vecA[2];
    vecC[2] = vecA[0]*vecB[1]-vecB[0]*vecA[1];
}

void VectorMag(float (&vecA)[3], float& magOut)
{
    VectorDotProduct(vecA,vecA,magOut);
    magOut = sqrt(magOut);
}

void VectorNorm(float (&vecA)[3])
{
    float mag(0.0);

    VectorMag(vecA,mag);
    vecA[0] /= mag;
    vecA[1] /= mag;
    vecA[2] /= mag;
}
