#include "A1_controller.h"
#include "ros/ros.h"
#include "kinematics.h"
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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "A1_testHAN");
    ros::NodeHandle nh;
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
          Leg(R_F, init_pos[0], signals[0])
        , Leg(L_F, init_pos[1], signals[1])
        , Leg(R_H, init_pos[2], signals[2])
        , Leg(L_H, init_pos[3], signals[3])
        };

    for(float x = 0; x < L2 / 1.5; x = x + 5){
        float y = 0;
        float z = -4/3 * L1;
        for(int i = 0; i < 4; i++){
            leg[i].setCooridinate(x, y, z);
        }
        // ROS_INFO("x=%f, y=%f, z=%f", x, y, z);
        ros::Duration(2).sleep();
    }
}