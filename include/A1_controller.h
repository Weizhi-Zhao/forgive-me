#pragma once
#include "serialPort/SerialPort.h"
#include <csignal>
#include <cmath>
#include "data_stream.h"
#include "ros/ros.h"

#define TEST_MODE 0
#define TORQUE_MODE 10
#define MAX_W 4.0
#define MAX_T 0.4

#define R_F "/dev/ttyUSB0"
#define L_F "/dev/ttyUSB1"
#define R_H "/dev/ttyUSB2"
#define L_H "/dev/ttyUSB3"

#define RF_HAA_INIT_POS 0.563739
#define RF_HFE_INIT_POS 3.649732
// #define RF_KFE_INIT_POS 14.865273
#define RF_KFE_INIT_POS -13.723231

#define LF_HAA_INIT_POS 0.588666
#define LF_HFE_INIT_POS 5.955311
#define LF_KFE_INIT_POS 18.730146
// #define LF_KFE_INIT_POS -11.235099

#define RH_HAA_INIT_POS 5.451014
#define RH_HFE_INIT_POS 6.133253
// #define RH_KFE_INIT_POS 17.47573
#define RH_KFE_INIT_POS -11.11276

#define LH_HAA_INIT_POS 4.132170
#define LH_HFE_INIT_POS 4.340025
#define LH_KFE_INIT_POS 15.234579
// #define LH_KFE_INIT_POS -13.353915

// #define RH_KFE_INIT_POS -12.341867



class A1_motor{
public:

    A1_motor(std::string Serial_Port, int id, float _initPos, int _signal): serial(Serial_Port){
        motor_run.id = id;
        motor_run.motorType = MotorType::A1Go1;
        initPos = _initPos;
        signal = _signal;
        setPID();
    }

    void getMotorData(MotorState& state){
        while(!serial.sendRecv(&motor_run, &motor_r));
        extract_data(&motor_r);
        state.q = (motor_r.Pos - initPos) / 9.1 * signal;
        // state.q = motor_r.Pos;
        state.dq = motor_r.W / 9.1 * signal;
        state.tauEst = motor_r.T * 9.1 * signal;
    }
    
    
    void setMotor(MotorCmd cmd){
        motor_run.mode = cmd.mode;
        motor_run.T = cmd.tau / 9.1 * signal;

        motor_run.T = std::min(double(motor_run.T), MAX_T);
        motor_run.T = std::max(double(motor_run.T), -MAX_T);

        motor_run.W = cmd.dq * 9.1 * signal;

        motor_run.W = std::min(double(motor_run.W), MAX_W);
        motor_run.W = std::max(double(motor_run.W), -MAX_W);

        motor_run.Pos = (cmd.q * 9.1 * signal+ initPos);
        setPID(cmd.Kp, cmd.Kd);
        modify_data(&motor_run);
        while(!serial.sendRecv(&motor_run, &motor_r));
    }
//  在关节电机的混合控制中，使用 PD 控制器将电机在输出位置的偏差反馈到力矩输出上：
//  τ = τ f f + k p · (p des − p) + k d · (ω des − ω)
    void setPID(){
        motor_run.K_P = 0.;
        motor_run.K_W = 0.;
    }

    void setPID(float K_P,float K_W){
//        设置限额
        if(K_P<0){
            K_P = 0;
        }
        if(K_P>16){
            K_P = 16;
        }
        if(K_W<0){
            K_W = 0;
        }
        if(K_W>32){
            K_W = 32;
        }
        motor_run.K_P = K_P;
        motor_run.K_W = K_W;
    }

private:
    SerialPort serial;
    MOTOR_send motor_run;
    MOTOR_recv motor_r;
    float initPos;
    int signal; //指示电机转动方向
    //控制器的state.q和cmd.q都是解算好的电机角度，因此正负之间的差别只需修改这两个的正负
};