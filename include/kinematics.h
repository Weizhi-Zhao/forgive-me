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
    Leg(string Serial_Port, float _initPos[3], int _signal[3])
        : motors[0](Serial_Port, 0, _initPos[0], _signal[0])
        , motors[1](Serial_Port, 1, _initPos[1], _signal[1])
        , motors[2](Serial_Port, 2, _initPos[2], _signal[2]){
    }
    
    void setCooridinate(float coordinate[3]);
    void setCooridinate(float x, float y, float z);
private:
    A1_motor motors[3];
    void InverseKinematics(float coordinate[3], double& angles[3]);
}