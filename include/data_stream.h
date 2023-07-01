#pragma once

#include <stdint.h>
using namespace std;

#pragma pack(1)
typedef struct
{
    float quaternion[4];     // quaternion, normalized, (w,x,y,z)
    float gyroscope[3];      // angular velocity （unit: rad/s)    (raw data)
    float accelerometer[3];  // m/(s2)                             (raw data)
    float rpy[3];            // euler angle（unit: rad)
} IMU;

typedef struct
{
    float q;       // current angle (unit: radian)
    float dq;      // current velocity (unit: radian/second)
    float tauEst;  // current estimated output torque (unit: N.m)
} MotorState;

typedef struct
{
    uint8_t mode;  // desired working mode
    float q;       // desired angle (unit: radian)
    float dq;      // desired velocity (unit: radian/second)
    float tau;     // desired output torque (unit: N.m)
    float Kp;      // desired position stiffness (unit: N.m/rad )
    float Kd;      // desired velocity stiffness (unit: N.m/(rad/s) )
} MotorCmd;

typedef struct
{
  IMU imu;
  MotorState motorState[12];
  //int16_t footForce[4];        // force sensors
} State;

typedef struct
{
  MotorCmd motorCmd[12];
} Cmd;

#pragma pack()
