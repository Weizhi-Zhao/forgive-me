#include "pd_force.h"
#include "ros/ros.h"

const float L1=20, L2=17.503;

void Pd_force::update_torque(float& tor1, float& tor2, float t1, float t2, 
                             float o1, float o2, float _target_speed, float _target_height){
    this->target_speed = _target_speed;
    this->target_height = _target_height;
    
    read_motor(t1, t2, o1, o2);
    calcuJacobi();
    forwardKinematics();
    calcuForce();
    calcuTorque();

    tor1 = this->torque1;
    tor2 = this->torque2;
}

void Pd_force::init(float t1, float t2, float o1, float o2){
    read_motor(t1, t2, o1, o2);
    calcuJacobi();
    forwardKinematics();

    last_t = ros::Time::now().toSec();
    last_vX = vX;
}

void Pd_force::read_motor(float t1, float t2, float o1, float o2){
    this->theta1 = t1;
    this->theta2 = t2;
    this->omega1 = o1;
    this->omega2 = o2;
}

void Pd_force::calcuJacobi(){
    jacobi[0][0] = L1 * cos(theta1) + L2 * cos(theta1 + theta2);
    jacobi[0][1] = L2 * cos(theta1 + theta2);
    jacobi[1][0] = L1 * sin(theta1) + L2 * sin(theta1 + theta2);
    jacobi[1][1] = L2 * sin(theta1 + theta2);
}

void Pd_force::forwardKinematics(){
    X = L1 * sin(theta1) + L2 * sin(theta1 + theta2);
    Z = -L1 * cos(theta1) - L2 * cos(theta1 + theta2);
    vX = jacobi[0][0] * omega1 + jacobi[0][1] * omega2;
    vZ = jacobi[1][0] * omega1 + jacobi[1][1] * omega2;
}

void Pd_force::calcuForce(){
    double now_t = ros::Time::now().toSec();
    double dt = now_t - last_t;
    float aX = (vX - last_vX) / dt;

    force_X = kp_X * (-target_speed - vX) + kd_X * (0 - aX);
    force_Z = kp_Z * (-target_height - Z) + kd_Z * (0 - vZ);

    last_vX = vX;
    last_t = now_t;
}

void Pd_force::calcuTorque(){
    torque1 = force_X * jacobi[0][0] + force_Z * jacobi[1][0];
    torque2 = force_X * jacobi[0][1] + force_Z * jacobi[1][1];
}

Pd_force::Pd_force(float _kp_X, float _kp_Z, float _kd_X, float _kd_Z){
    this->kp_X = _kp_X;
    this->kp_Z = _kp_Z;
    this->kd_X = _kd_X;
    this->kd_Z = _kd_Z;
}