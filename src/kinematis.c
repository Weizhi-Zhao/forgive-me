#include "kinematis.h"

// 运动学反解
void Leg::InverseKinematics(float coordinate[3], double& angles[3]){

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
                                                / (2 * sqrt( pow(x, 2) +vpow(z, 2) ) * L1)
                                            );

    angles[0] = 0;
}

void Leg::setCooridinate(float coordinate[3]){
    float angle[3];
    InverseKinematics(coordinate, angle);
    MotorCmd cmd;
    cmd.mode = 10;  // desired working mode
    cmd.q;       // desired angle (unit: radian)
    cmd.dq = 0;      // desired velocity (unit: radian/second)
    cmd.tau = 0;     // desired output torque (unit: N.m)
    cmd.Kp = 0.3;      // desired position stiffness (unit: N.m/rad )
    cmd.Kd = 0.3;      // desired velocity stiffness (unit: N.m/(rad/s) )
    for(int i = 0; i < 3; i++){
        cmd.q = angle[i];       // desired angle (unit: radian)
        motor[i].setMotor(cmd);
    }
}

void Leg::setCooridinate(float x, float y, float z){
    float angle[3];
    float coordinate[3] = {x, y, z};
    InverseKinematics(coordinate, angle);
    MotorCmd cmd;
    cmd.mode = 10;  // desired working mode
    cmd.q;       // desired angle (unit: radian)
    cmd.dq = 0;      // desired velocity (unit: radian/second)
    cmd.tau = 0;     // desired output torque (unit: N.m)
    cmd.Kp = 0.3;      // desired position stiffness (unit: N.m/rad )
    cmd.Kd = 0.3;      // desired velocity stiffness (unit: N.m/(rad/s) )
    for(int i = 0; i < 3; i++){
        cmd.q = angle[i];       // desired angle (unit: radian)
        motor[i].setMotor(cmd);
    }
}