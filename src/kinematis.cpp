#include "kinematics.h"
#include "ros/ros.h"

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

void Leg::set_stepSize(float expect_stepSize){
    this->target_stepSize = expect_stepSize;
}

void Leg::trot(const int nowPhase, const float diff){
    float coordinate[3];

    // 根据目标步长，**温柔的**调整实际步长
    if(this->target_stepSize > this->stepSize + this->dl){
        this->stepSize += this->dl;
    }else if(this->target_stepSize < this->stepSize - this->dl){
        this->stepSize -= this->dl;
    }else{
        this->stepSize = this->target_stepSize;
    }

    generateTrajectory(nowPhase, coordinate);
    setCooridinate(coordinate);
}

void Leg::generateTrajectory(int nowPhase, float coordinate[3]){
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

    if(nowPhase < this->phaseNum / 2){
        // 抬腿
        x = this->stepLength * ( t  - 0.5 / PI * sin(2*PI*t) );
        y = 0;
        z = this->upLength * ( 0.5 - 0.5 * cos(2*PI*t) ) - L3;
    }
    else{
        // 落腿
        x = this->stepLength * ( 1 - t  + 0.5 / PI * sin(2*PI*t) );
        y = 0;
        z = this->downLength * ( 0.5 - 0.5 * cos(2*PI*t) ) - L3;

        // 水平落脚代码（没测过不敢保证对，但是应该试试）
        /*
        x = this->stepLength * ( 1 - t  + 0.5 / PI * sin(2*PI*t) );
        // x = this->stepLength * ( 1 - t ); //也说不定线性的更好？
        y = 0;
        z = - L3;
        */
    }
}