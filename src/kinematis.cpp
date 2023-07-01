#include "kinematics.h"
#include "ros/ros.h"

float Leg::upLength[3] = {0, 0, 0};
float Leg::downLength[3] = {0, 0, 0};

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
    angles[1] = PI / 2 - atan(-z / x) - acos(
                                                (pow(x, 2) + pow(z, 2) + pow(L1, 2) - pow(L2, 2))
                                                / (2 * sqrt( pow(x, 2) + pow(z, 2) ) * L1)
                                            );

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
    cmd.Kp = 0.3;      // desired position stiffness (unit: N.m/rad )
    cmd.Kd = 0.3;      // desired velocity stiffness (unit: N.m/(rad/s) )
    for(int i = 0; i < 3; i++){
        cmd.q = angle[i];       // desired angle (unit: radian)
        motors[i].setMotor(cmd);
    }
}

void Leg::setCooridinate(float x, float y, float z){
    float angle[3];
    float coordinate[3] = {x, y, z};
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
        ROS_INFO("%d angle: %f", i, cmd.q);
    }
}

void Leg::trot(int nowPhase){
    float coordinate[3];
    generateTrajectory(nowPhase, coordinate);
    setCooridinate(coordinate);
}

void Leg::generateTrajectory(int phase, float coordinate[3])
{
    float& x = coordinate[0];
    float& y = coordinate[1];
    float& z = coordinate[2];

    // 把腿偏置到属于它的周期
    phase = (phase + self.phaseBias) % self.phaseNum;

    // 如果把总周期的一半当成子周期，subPhase就是子周期的相位
    float subPhase;
    if(phase > self.phaseNum / 2){
        subPhase = phase - self.phaseNum / 2;
    }
    else{
        subPhase = phase;
    }

    // 把子相位缩放到2Pi里
    float theta = subPhase / (self.phaseNum / 2 - 1) * 2 * PI;
    float t = subPhase / (self.phaseNum / 2 - 1);

    if(phase < self.phaseNum / 2){
        // 抬腿
        x = self.upLength[0] * ( t  - 0.5 / PI * sin(2*PI*t) );
        y = self.upLength[1] * ( 0.5 - 0.5 * cos(2*PI*t) ) - L3;
        z = 0;
    }
    else{
        // 落腿
        x = self.downLength[0] * ( 1 - t  + 0.5 / PI * sin(2*PI*t) );
        y = self.downLength[1] * ( 0.5 - 0.5 * cos(2*PI*t) ) - L3;
        z = 0;
    }
}