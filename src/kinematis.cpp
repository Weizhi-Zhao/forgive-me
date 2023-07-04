#include "kinematics.h"
#include "ros/ros.h"
#include <math.h>

Leg::DYNAMIC_KP = {
                    {0.005, 0.003, 0.003},
                    {0.01 , 0.006, 0.006},
                    {0.02 , 0.01 , 0.01 },
                    {0.01 , 0.006, 0.006},
                  };

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

void Leg::setCooridinate(float coordinate[3], float kp[3]){
    float angle[3];
    inverseKinematics(coordinate, angle);
    MotorCmd cmd;
    cmd.mode = 10;  // desired working mode
    cmd.q;       // desired angle (unit: radian)
    cmd.dq = 0;      // desired velocity (unit: radian/second)
    cmd.tau = 0;     // desired output torque (unit: N.m)
    // cmd.Kp = 0.06;      // desired position stiffness (unit: N.m/rad )
    cmd.Kd = 3;      // desired velocity stiffness (unit: N.m/(rad/s) )
    for(int i = 0; i < 3; i++){
        cmd.q = angle[i];       // desired angle (unit: radian)
        cmd.Kp = kp[i];
        motors[i].setMotor(cmd);
    }
}

void Leg::setCooridinate(float x, float y, float z, float kp[3]){
    float coordinate[3] = {x, y, z};
    setCooridinate(coordinate, kp);
}

void Leg::set_step(float expect_stepSize, float expect_upLength, float expect_downLength){
    this->target_stepSize = expect_stepSize;
    this->target_upLength = expect_upLength;
    this->target_downLength = expect_downLength;
}


void Leg::trot(const int nowPhase){
    float coordinate[3];
    int kp_id;

    // 根据目标步长，**温柔的**调整实际参数
    graduallyApproching(this->target_stepSize, this->stepSize, dl);
    graduallyApproching(this->target_upLength, this->upLength, dUp);
    graduallyApproching(this->target_downLength, this->downLength, dDown);

    // 根据相位生成kp，和轨迹
    generateTrajectory(nowPhase, coordinate, kp_id);
    setCooridinate(coordinate, Leg::DYNAMIC_KP[kp_id]);
}

void graduallyApproching(float target, float& actual, float d){
    if(target > actual + d){
        actual += d;
    }else if(target < actual - d){
        actual -= d;
    }else{
        actual = target;
    }
}

void Leg::generateTrajectory(int nowPhase, float coordinate[3], int& kp_id){
    // 把腿偏置到属于它的周期
    nowPhase = (nowPhase + this->phaseBias) % this->phaseNum;

    float subPhase = nowPhase;
    int chunk_id = 0;
    for(chunk_id = 0; chunk_id < 4; chunk_id++){
        if(subPhase >= this->cycleChunking[chunk_id]){
            subPhase -= this->cycleChunking[chunk_id];
        }else{
            break;
        }
    }

    float t = subPhase / (this->cycleChunking[chunk_id] - 1);

    if(chunk_id == 0){
        swing_phase(t, coordinate);
    }else if(chunk_id == 1){
        stop_phase_1(t, coordinate);
    }else if(chunk_id == 2){
        prop_phase(t, coordinate);
    }else if(chunk_id == 3){
        stop_phase_2(t, coordinate);
    }

    kp_id = chunk_id;
}

void Leg::swing_phase(const float t, float coordinate[3]){
    float& x = coordinate[0];
    float& y = coordinate[1];
    float& z = coordinate[2];

    x = this->stepSize * ( t  - 0.5 / PI * sin(2*PI*t) - 0.5) + abs(this->stepSize) / 2;
    y = 0;
    z = this->upLength * ( 0.5 - 0.5 * cos(2*PI*t) ) - L3;
}

void Leg::prop_phase(const float t, float coordinate[3]){
    float& x = coordinate[0];
    float& y = coordinate[1];
    float& z = coordinate[2];

    x = this->stepSize * ( 1 - t  + 0.5 / PI * sin(2*PI*t) -0.5) + abs(this->stepSize) / 2;
    y = 0;
    z = this->downLength * ( 0.5 - 0.5 * cos(2*PI*t) ) - L3;

    // 水平落脚代码（没测过不敢保证对，但是应该试试）
    // x = this->stepSize * ( 1 - t  + 0.5 / PI * sin(2*PI*t) - 0.5) + abs(this->stepSize) / 2;
    // // x = this->stepSize * ( 1 - t ); //也说不定线性的更好？
    // y = 0;
    // z = - L3;
}

// 摆动相之后的停顿
void Leg::stop_phase_1(const float t, float coordinate[3]){
    float& x = coordinate[0];
    float& y = coordinate[1];
    float& z = coordinate[2];

    x = this->stepSize * (0.5) + abs(this->stepSize) / 2;
    y = 0;
    z = -L3;
}

// 支撑相之后的停顿
void Leg::stop_phase_2(const float t, float coordinate[3]){
    float& x = coordinate[0];
    float& y = coordinate[1];
    float& z = coordinate[2];

    x = this->stepSize * (-0.5) + abs(this->stepSize) / 2;
    y = 0;
    z = -L3;
}
