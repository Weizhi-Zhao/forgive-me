#include "A1_controller.h"
#include "ros/ros.h"
#include "kinematics.h"
#include "std_msgs/String.h"
#include <string>
// int main()
// {
//     // A1_motor motor("/dev/ttyUSB2", 2, RH_KFE_INIT_POS, -1);
//     A1_motor* motor[12];
//     motor[0]  = new A1_motor(R_F, 0, RF_HAA_INIT_POS,  1);
//     motor[1]  = new A1_motor(R_F, 1, RF_HFE_INIT_POS, -1);
//     motor[2]  = new A1_motor(R_F, 2, RF_KFE_INIT_POS, -1);
//     motor[3]  = new A1_motor(L_F, 0, LF_HAA_INIT_POS,  1);
//     motor[4]  = new A1_motor(L_F, 1, LF_HFE_INIT_POS,  1);
//     motor[5]  = new A1_motor(L_F, 2, LF_KFE_INIT_POS,  1);
//     motor[6]  = new A1_motor(R_H, 0, RH_HAA_INIT_POS, -1);
//     motor[7]  = new A1_motor(R_H, 1, RH_HFE_INIT_POS, -1);
//     motor[8]  = new A1_motor(R_H, 2, RH_KFE_INIT_POS, -1);
//     motor[9]  = new A1_motor(L_H, 0, LH_HAA_INIT_POS, -1);
//     motor[10] = new A1_motor(L_H, 1, LH_HFE_INIT_POS,  1);
//     motor[11] = new A1_motor(L_H, 2, LH_KFE_INIT_POS,  1);

//     MotorCmd cmd;
//     cmd.mode = TEST_MODE;
//     cmd.Kp = 0;
//     cmd.Kd = 3;
//     cmd.q = 0.602089;
//     cmd.dq = 2;
//     cmd.tau = -3.84;
//     long long cnt;
//     // motor.setMotor(cmd);
//     MotorState state;
//     while(1){
//         for(int i = 0; i<12; i++)
//         {
//             // motor[i]->setMotor(cmd);
//             motor[i]->getMotorData(state);
//             if(cnt % 50 == 0) ROS_INFO("%d pos: %f", i, state.q);
//         }
//         if(cnt % 50 == 0) 
//         ROS_INFO("\n");
//         cnt++;
//     }
//     return 0;
// }

const int PHASE_NUM = 16;
const float max_stepSize = 10; //设置最大步长
std::string command;

void key_callback(const std_msgs::String::ConstPtr& msg){
    command = msg->data;
}

void forward(Leg& leg[4]){
    leg[0].set_stepSize(max_stepSize);
    leg[2].set_stepSize(max_stepSize);
    leg[1].set_stepSize(max_stepSize);
    leg[3].set_stepSize(max_stepSize);
}

void backward(Leg& leg[4]){
    leg[0].set_stepSize(-max_stepSize);
    leg[2].set_stepSize(-max_stepSize);
    leg[1].set_stepSize(-max_stepSize);
    leg[3].set_stepSize(-max_stepSize);
}

void stop(Leg& leg[4]){
    leg[0].set_stepSize(0);
    leg[2].set_stepSize(0);
    leg[1].set_stepSize(0);
    leg[3].set_stepSize(0);
}

void turn_right(Leg& leg[4]){
    // 右
    leg[0].set_stepSize(-max_stepSize);
    leg[2].set_stepSize(-max_stepSize);
    // 左
    leg[1].set_stepSize(max_stepSize);
    leg[3].set_stepSize(max_stepSize);
}

void turn_left(Leg& leg[4]){
    // 右
    leg[0].set_stepSize(max_stepSize);
    leg[2].set_stepSize(max_stepSize);
    // 左
    leg[1].set_stepSize(-max_stepSize);
    leg[3].set_stepSize(-max_stepSize);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "A1_forgiveme");
    ros::NodeHandle nh;
    ros::Subscriber key_sub = nh.subscribe("/key_controller", 10, key_callback);
    float init_pos[4][3] = {
              {RF_HAA_INIT_POS, RF_HFE_INIT_POS, RF_KFE_INIT_POS}
            , {LF_HAA_INIT_POS, LF_HFE_INIT_POS, LF_KFE_INIT_POS}
            , {RH_HAA_INIT_POS, RH_HFE_INIT_POS, RH_KFE_INIT_POS}
            , {LH_HAA_INIT_POS, LH_HFE_INIT_POS, LH_KFE_INIT_POS}
        };
    int signals[4][3] = {
                  { 1,  1,  1}
                , { 1, -1, -1}
                , {-1,  1,  1}
                , {-1, -1, -1}
            };
    Leg leg[4] = {
          Leg(0, R_F, init_pos[0], signals[0], true)
        , Leg(1, L_F, init_pos[1], signals[1], false)
        , Leg(2, R_H, init_pos[2], signals[2], false)
        , Leg(3, L_H, init_pos[3], signals[3], true)
        };

    // 站立的代码。
    for(int i = 0; i < 4; i++){
       leg[i].setCooridinate(0, 0, -L3);
    }

    int phase = 0;
    bool p_presed = false; //是否按下p（是否开始踏步）
    // 根据command判断要执行的步态
    while(ros::ok()){
        ros::spinOnce(); // 网上说加上这句才能回调函数
        if(command == 'pp'){
            // p: play
            // 按p开始踏步
            stop(leg);
            for(int i = 0; i < PHASE_NUM / 2; i++)
            {
                leg[0].trot(i);
                leg[3].trot(i);
                ros::Duration(0.02).sleep();
            }
            for(int i = PHASE_NUM / 2; i < PHASE_NUM; i++)
            {
                leg[0].trot(i);
                leg[3].trot(i);
                leg[1].trot(i);
                leg[2].trot(i);
                ros::Duration(0.02).sleep();
            }
            p_presed = true;
        }else if(p_presed == false){
            // 如果开没有开始踏步，就跳过后面的
            continue;
        }if(command == "wp"){
            // 按下w
            // ROS_INFO("按下w");
            forward(leg);
        }else if(command == "wr"){
            // 松开w
            stop(leg);
        }else if(command == "ap"){
            // 按下a
            turn_left(leg);
        }else if(command == "ar"){
            // 松开a
        }else if(command == "sp"){
            // 按下s
            backward(leg);
        }else if(command == "sr"){
            // 松开s
            stop(leg);
        }else if(command == "dp"){
            // 按下d
            turn_left(leg);
        }else if(command == "dr"){
            // 松开d
            stop(leg);
        }else{
            stop(leg);
        }

        // trot
        for(int j = 0; j < 4; j++){
            leg[j].trot(phase);
        }
        phase = (phase + 1) % PHASE_NUM;
        ros::Duration(0.02).sleep();
    }
}