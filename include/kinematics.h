#include <csignal>
#include <cmath>
#include <iostream>
#include "A1_controller.h"
#include "data_stream.h"
using namespace std;

const float L1=20, L2=17.503, L3=17;
const float PI = 3.1415926;

class Leg{
public:
    Leg(int _id, string Serial_Port, float _initPos[3], int _signal[3], bool _phaseBias, int _phaseNum = 16)
        : motors{ A1_motor(Serial_Port, 0, _initPos[0], _signal[0])
                , A1_motor(Serial_Port, 1, _initPos[1], _signal[1])
                , A1_motor(Serial_Port, 2, _initPos[2], _signal[2])}{
        id = _id;
        phaseNum = _phaseNum;
        // 如果该腿相位落后的话，相位偏置为半个周期
        phaseBias = _phaseBias ? phaseNum / 2 : 0;
    }
    
    void setCooridinate(float coordinate[3]);
    void setCooridinate(float x, float y, float z);
    void trot(const int nowPhase, const float diff);

private:
    int id;
    A1_motor motors[3];
    // 总点数，一般有16个点，前8个点为抬腿，后8个点为落腿
    int phaseNum;
    // 相位偏置，有一对腿（对角线）相位比另一对腿，落后半个周期
    int phaseBias;

    static float upLength[4];
    static float downLength[4];
    static float stepLength[4];

    void generateTrajectory(int phase, float coordinate[3], const float diff);

    void inverseKinematics(const float coordinate[3], float angles[3]);
};