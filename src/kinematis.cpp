#include "kinematics.h"
#include "ros/ros.h"

float Leg::upLength[4] = {10, 10, 10, 10};
float Leg::downLength[4] = {-1, -1, -1, -1};
float Leg::stepLength[4] = {10, 10, 10, 10};

// 运动学反解
void Leg::inverseKinematics(const float coordinate[3], float angles[3]){

    float x = coordinate[0];
    float y = coordinate[1];
    float z = coordinate[2];

    // 向上为正
    angles[2] = PI - acos(
                            ( pow(x, 2) + pow(z, 2) - pow(L1, 2) - pow(L2, 2) ) 
                            / (-2 * L1 * L2)
                         );
                    
    // 向前为正
    if(x >= 0){
        angles[1] = PI / 2 - atan(-z / x) - acos(
                                                    (pow(x, 2) + pow(z, 2) + pow(L1, 2) - pow(L2, 2))
                                                    / (2 * sqrt( pow(x, 2) + pow(z, 2) ) * L1)
                                                );
    }else{
        angles[1] = -PI / 2 - atan(-z / x) - acos(
                                                    (pow(x, 2) + pow(z, 2) + pow(L1, 2) - pow(L2, 2))
                                                    / (2 * sqrt( pow(x, 2) + pow(z, 2) ) * L1)
                                                );
    }
    

    angles[0] = 0;
}

void Leg::setCooridinate(float coordinate[3]){
    float angle[3];
    inverseKinematics(coordinate, angle);
    MotorCmd cmd;
    cmd.mode = 10;  // desired working mode
    cmd.q;       // desired angle (unit: radian)
    cmd.dq = 0;      // desired velocity (unit: radian/second)
    cmd.tau = 0;     // desired output torque (unit: N.m)
    cmd.Kp = 0.002;      // desired position stiffness (unit: N.m/rad )
    cmd.Kd = 0.002;      // desired velocity stiffness (unit: N.m/(rad/s) )
    for(int i = 0; i < 3; i++){
        cmd.q = angle[i];       // desired angle (unit: radian)
        motors[i].setMotor(cmd);
    }
}

void Leg::setCooridinate(float x, float y, float z){
    float coordinate[3] = {x, y, z};
    setCooridinate(coordinate);
}

void Leg::trot(const int nowPhase, const float diff){
    float coordinate[3];
    generateTrajectory(nowPhase, coordinate, diff);
    setCooridinate(coordinate);
}

// diff 代表左边步长的放大系数，右边步长的缩小系数
void Leg::generateTrajectory(int nowPhase, float coordinate[3], const float diff){
    float& x = coordinate[0];
    float& y = coordinate[1];
    float& z = coordinate[2];

    // 把腿偏置到属于它的周期
    nowPhase = (nowPhase + this->phaseBias) % this->phaseNum;

    // 如果把总周期的一半当成子周期，subPhase就是子周期的相位
    float subPhase;
    if(nowPhase >= this->phaseNum / 2){
        subPhase = nowPhase - this->phaseNum / 2;
    }
    else{
        subPhase = nowPhase;
    }

    // 把子相位缩放到2Pi里
    float t = subPhase / (this->phaseNum / 2 - 1);

    // diff 代表左边步长的放大系数，右边步长的缩小系数
    float scale = (this->id % 2 == 0) ? (1/diff) : diff;

    if(nowPhase < this->phaseNum / 2){
        // 抬腿
        x = scale * Leg::stepLength[id] * ( t  - 0.5 / PI * sin(2*PI*t) );
        y = 0;
        z = Leg::upLength[id] * ( 0.5 - 0.5 * cos(2*PI*t) ) - L3;
    }
    else{
        // 落腿
        x = scale * Leg::stepLength[id] * ( 1 - t  + 0.5 / PI * sin(2*PI*t) );
        y = 0;
        z = Leg::downLength[id] * ( 0.5 - 0.5 * cos(2*PI*t) ) - L3;
    }
}