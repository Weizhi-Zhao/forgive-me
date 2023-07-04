#include "A1_controller.h"
#include "ros/ros.h"
#include "kinematics.h"
#include "std_msgs/String.h"
#include <string>

const int CYCLE_CHUNKING[4] = {16, 10, 16, 10};
const int PHASE_NUM = CYCLE_CHUNKING[0] + CYCLE_CHUNKING[1] + CYCLE_CHUNKING[2] + CYCLE_CHUNKING[3];
const float MAX_STEPSIZE = 5; //设置最大步长
const float DELAY_TIME = 0.002;
const float MAX_UPLENGTH = 5, MAX_DOWNLENGTH = -1;

std::string command;

void key_callback(const std_msgs::String::ConstPtr& msg){
    command = msg->data;
}

void forward(Leg leg[4]){
    leg[0].set_step(MAX_STEPSIZE, MAX_UPLENGTH, MAX_DOWNLENGTH);
    leg[2].set_step(MAX_STEPSIZE, MAX_UPLENGTH, MAX_DOWNLENGTH);
    leg[1].set_step(MAX_STEPSIZE, MAX_UPLENGTH, MAX_DOWNLENGTH);
    leg[3].set_step(MAX_STEPSIZE, MAX_UPLENGTH, MAX_DOWNLENGTH);
}

void backward(Leg leg[4]){
    leg[0].set_step(-MAX_STEPSIZE, MAX_UPLENGTH, MAX_DOWNLENGTH);
    leg[2].set_step(-MAX_STEPSIZE, MAX_UPLENGTH, MAX_DOWNLENGTH);
    leg[1].set_step(-MAX_STEPSIZE, MAX_UPLENGTH, MAX_DOWNLENGTH);
    leg[3].set_step(-MAX_STEPSIZE, MAX_UPLENGTH, MAX_DOWNLENGTH);
}

void stop(Leg leg[4]){
    leg[0].set_step(0, MAX_UPLENGTH, MAX_DOWNLENGTH);
    leg[2].set_step(0, MAX_UPLENGTH, MAX_DOWNLENGTH);
    leg[1].set_step(0, MAX_UPLENGTH, MAX_DOWNLENGTH);
    leg[3].set_step(0, MAX_UPLENGTH, MAX_DOWNLENGTH);
}

void turn_right(Leg leg[4]){
    // 右
    leg[0].set_step(-MAX_STEPSIZE, MAX_UPLENGTH, MAX_DOWNLENGTH);
    leg[2].set_step(-MAX_STEPSIZE, MAX_UPLENGTH, MAX_DOWNLENGTH);
    // 左
    leg[1].set_step(MAX_STEPSIZE, MAX_UPLENGTH, MAX_DOWNLENGTH);
    leg[3].set_step(MAX_STEPSIZE, MAX_UPLENGTH, MAX_DOWNLENGTH);
}

void turn_left(Leg leg[4]){
    // 右
    leg[0].set_step(MAX_STEPSIZE, MAX_UPLENGTH, MAX_DOWNLENGTH);
    leg[2].set_step(MAX_STEPSIZE, MAX_UPLENGTH, MAX_DOWNLENGTH);
    // 左
    leg[1].set_step(-MAX_STEPSIZE, MAX_UPLENGTH, MAX_DOWNLENGTH);
    leg[3].set_step(-MAX_STEPSIZE, MAX_UPLENGTH, MAX_DOWNLENGTH);
}

void stand(Leg leg[4]){
    for(int i = 0; i < 4; i++){
        leg[i].set_step(0, 0, 0);
    }
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
          Leg(0, R_F, init_pos[0], signals[0], true, CYCLE_CHUNKING)
        , Leg(1, L_F, init_pos[1], signals[1], false, CYCLE_CHUNKING)
        , Leg(2, R_H, init_pos[2], signals[2], false, CYCLE_CHUNKING)
        , Leg(3, L_H, init_pos[3], signals[3], true, CYCLE_CHUNKING)
        };

    int phase;
    bool p_pressed;
    while(ros::ok()){

        // 站立的代码。
        for(int i = 0; i < 4; i++){
            leg[i].setCooridinate(0, 0, -L3);
        }

        ros::Duration(0.2).sleep();

        phase = 0;
        p_pressed = false; //是否按下p（是否开始踏步）
        // 根据command判断要执行的步态
        while(ros::ok()){
            ros::spinOnce(); // 网上说加上这句才能回调函数
            if(command == "pp" && p_pressed == false){
                // p: play
                // 按p开始踏步
                stop(leg);
                for(int i = 0; i < PHASE_NUM / 2; i++)
                {
                    leg[1].trot(i);
                    leg[2].trot(i);
                    ros::Duration(DELAY_TIME).sleep();
                }
                for(int i = PHASE_NUM / 2; i < PHASE_NUM; i++)
                {
                    leg[0].trot(i);
                    leg[3].trot(i);
                    leg[1].trot(i);
                    leg[2].trot(i);
                    ros::Duration(DELAY_TIME).sleep();
                }
                p_pressed = true;
            }else if(p_pressed == false){
                // 如果开没有开始踏步，就跳过后面的
                continue;
            }else if(command == "wp"){
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
                stop(leg);
            }else if(command == "sp"){
                // 按下s
                backward(leg);
            }else if(command == "sr"){
                // 松开s
                stop(leg);
            }else if(command == "dp"){
                // 按下d
                turn_right(leg);
            }else if(command == "dr"){
                // 松开d
                stop(leg);
            }else if(command == "kp" || command == "kr"){
                // k: kill
                // 停止踏步
                stand(leg);
                // break;
            }else{
                stop(leg);
            }

            // trot
            for(int j = 0; j < 4; j++){
                leg[j].trot(phase);
            }
            phase = (phase + 1) % PHASE_NUM;
            ros::Duration(DELAY_TIME).sleep();
        }

        // 站住
        // for(int i = phase; i < PHASE_NUM; i++){
        //     leg[0].trot(i);
        //     leg[3].trot(i);
        //     leg[1].trot(i);
        //     leg[2].trot(i);
        //     ros::Duration(DELAY_TIME).sleep();
        // }
        // for(int i = 0; i < PHASE_NUM / 2; i++){
        //     leg[0].trot(i);
        //     leg[3].trot(i);
        //     leg[1].trot(i);
        //     leg[2].trot(i);
        //     ros::Duration(DELAY_TIME).sleep();
        // }
        // for(int i = PHASE_NUM / 2; i < PHASE_NUM; i++)
        // {
        //     leg[1].trot(i);
        //     leg[2].trot(i);
        //     ros::Duration(DELAY_TIME).sleep();
        // }
    }
}
