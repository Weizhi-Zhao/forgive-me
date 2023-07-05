#pragma once

#include <csignal>
#include <cmath>
#include <iostream>
#include "A1_controller.h"
#include "data_stream.h"
#include "pd_force.h"
using namespace std;

const float L1=20, L2=17.503, L3=25;
const float PI = 3.1415926;

class Leg{
public:
    Leg(int _id, string Serial_Port, float _initPos[3], int _signal[3], bool _phaseBias, 
        int _cycleChunking[4], Pd_force _prop_forceController)
        : motors{   A1_motor(Serial_Port, 0, _initPos[0], _signal[0]), 
                    A1_motor(Serial_Port, 1, _initPos[1], _signal[1]), 
                    A1_motor(Serial_Port, 2, _initPos[2], _signal[2])},
          cycleChunking{_cycleChunking[0], _cycleChunking[1], 
                        _cycleChunking[2], _cycleChunking[3]},
          prop_forceController(_prop_forceController) {

        id = _id;

        // 计算总点数
        phaseNum = 0;
        for(int i = 0; i < 4; i++)
        {
            phaseNum += cycleChunking[i];
        }

        // 如果该腿相位落后的话，相位偏置为半个周期
        phaseBias = _phaseBias ? phaseNum / 2 : 0;

        // 设置初始的步态参数
        upLength = 0;
        downLength = 0;
        stepSize = 0;
        dl = 10.0 / phaseNum;
        dUp = 10.0 / phaseNum;
        dDown = 2.0/ phaseNum;
        target_stepSize = stepSize;
        target_upLength = upLength;
        target_downLength = downLength;
        testSignal = false;
    }
    
    void setCooridinate(float coordinate[3], float kp[3]);
    void setCooridinate(float x, float y, float z, float kp[3]);
    void trot(const int nowPhase);
    void set_step(float expect_stepSize, float expect_upLength, float expect_downLength);
    void testController(void);

private:
    int id;
    A1_motor motors[3];
    // 总点数，一般有16个点，前8个点为抬腿，后8个点为落腿
    int phaseNum;
    // 四个动作各占几个点（摆动、停止、支撑、停止）
    int cycleChunking[4];

    // 4个相位的，3个电机的kp
    const static float DYNAMIC_KP[4][3];

    // 相位偏置，有一对腿（对角线）相位比另一对腿，落后半个周期
    int phaseBias;

    // 每个腿有自己的步态参数
    float upLength;
    float downLength;
    float stepSize;

    // 目标步长（当前的步长不一定等于目标步长）
    // 当前步长会**缓慢**的接近目标步长
    float target_stepSize;
    float target_upLength;
    float target_downLength;
    float dl; //这是每个轨迹点的步长变化量（看代码更好懂）
    float dUp;
    float dDown;
    bool testSignal;

    Pd_force prop_forceController;

    // float lastCoordinate[3];

    void swing_phase(const float t, float coordinate[3]);

    // 力控的支撑相
    void force_propPhase();
    const static float PROP_TIME;

    void prop_phase(const float t, float coordinate[3]);

    // 摆动相之后的停顿
    void stop_phase_1(const float t, float coordinate[3]);

    // 支撑相之后的停顿
    void stop_phase_2(const float t, float coordinate[3]);


    // void generateTrajectory(int phase, float coordinate[3], int& kp_id);
    void generateTrajectory(int nowPhase);

    void inverseKinematics(const float coordinate[3], float angles[3]);

    void graduallyApproching(void);
};
