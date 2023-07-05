#pragma once

#include <math.h>

class Pd_force{
public:
    Pd_force(float _kp_X, float _kp_Z, float _kd_X, float _kd_Z);

    void update_torque(float& tor1, float& tor2, float t1, float t2,
                       float o1, float o2, float _target_speed, float _target_height);
    void init(float t1, float t2, float o1, float o2);

private:
    // ----------已知----------
    float theta1, theta2; // 角度
    float omega1, omega2; // 角速度
    float kp_X, kp_Z;
    float kd_X, kd_Z;
    float target_speed; // cm/s
    float target_height; // cm

    // --------计算得到--------
    float jacobi[2][2]; // 雅可比矩阵
    float vX, vZ; // 速度
    float X, Z; // 位置
    // float aX; // 加速度
    float force_X, force_Z; // 计算得到的力
    float torque1, torque2; // 计算得到的力矩
    float last_vX; // 用于计算aX
    double last_t; // 用于计算aX 还是高精度保险
    
    void read_motor(float t1, float t2, float o1, float o2);

    void forwardKinematics();

    void calcuForce();

    void calcuJacobi();

    void calcuTorque();
};