#include "ros/ros.h"
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>
#include "std_msgs/String.h"

#include "Eigen/Dense"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/StdVector"

#include "robot_msgs/Move.h"
#include "std_srvs/Empty.h"
#include "std_srvs/SetBool.h"
#include "robot_msgs/RobotMsg.h"
#include "robot_msgs/SetUserFrame.h"
#include "robot_msgs/SetTcp.h"
#include "robot_msgs/SetLoad.h"
#include "robot_msgs/ServoL.h"
#include "robot_msgs/ClearErr.h"
#include "robot_msgs/SetCollision.h"
#include "robot_msgs/SetAxis.h"
#include "robot_msgs/GetPosition.h"

#include "admittance_control/Plot.h"

#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"

#include <pthread.h>
#include <unistd.h>
#include <sstream>

#include <vector>

#include "libs/robot.h"
#include "libs/conversion.h"
#include "time.h"
#include <map>
#include <string>
#include <sys/time.h>
#include <cstdlib>
#include <cstdio>
#include <ctime>
#include <cmath>
#include <unistd.h>

using namespace std;
using namespace Eigen;

const double PI = 3.1415926;

int index_initial = 100;

std::map<int, string> mapErr = {
    {2, "ERR_FUCTION_CALL_ERROR"},
    {-1, "ERR_INVALID_HANDLER"},
    {-2, "ERR_INVALID_PARAMETER"},
    {-3, "ERR_COMMUNICATION_ERR"},
    {-4, "ERR_KINE_INVERSE_ERR"},
    {-5, "ERR_EMERGENCY_PRESSED"},
    {-6, "ERR_NOT_POWERED"},
    {-7, "ERR_NOT_ENABLED"},
    {-8, "ERR_DISABLE_SERVOMODE"},
    {-9, "ERR_NOT_OFF_ENABLE"},
    {-10, "ERR_PROGRAM_IS_RUNNING"},
    {-11, "ERR_CANNOT_OPEN_FILE"},
    {-12, "ERR_MOTION_ABNORMAL"}};

/* global variables */
JAKAZuRobot robot;

pthread_mutex_t mutex;

VectorXd expected_pose_servo(6);
VectorXd current_joint_servo(6);

bool servo_mode_open_flag = false;
bool servo_pose_change_flag = false;

admittance_control::Plot plot_data;

struct timeval tv;
// gettimeofday(&tv, NULL);
// std::cout << "begin" << tv.tv_sec << "s," << tv.tv_usec << "微秒" << endl;

// 1.1 service move line -
bool movel_callback(robot_msgs::Move::Request &req,
                    robot_msgs::Move::Response &res)
{
    CartesianPose cart;
    cart.tran.x = req.pose[0] * 1000;
    cart.tran.y = req.pose[1] * 1000;
    cart.tran.z = req.pose[2] * 1000;

    cart.rpy.rx = req.pose[3];
    cart.rpy.ry = req.pose[4];
    cart.rpy.rz = req.pose[5];

    double speed = (double)req.mvvelo * 1000;
    double accel = (double)req.mvacc * 1000;

    OptionalCond *p = nullptr;

    pthread_mutex_lock(&mutex);

    int sdk_res = robot.linear_move(&cart, MoveMode::ABS, req.is_block, speed, accel, 0.3, p);

    pthread_mutex_unlock(&mutex);
    switch (sdk_res)
    {
    case 0:
        res.ret = 1;
        res.message = "cmd has been executed!";
        break;
    default:
        res.ret = sdk_res;
        res.message = mapErr[sdk_res] + "\n";
        break;
    }

    return true;
}

// 1.2 service move joint -
bool movej_callback(robot_msgs::Move::Request &req,
                    robot_msgs::Move::Response &res)
{
    JointValue joint_pose;
    joint_pose.jVal[0] = (req.pose[0]);
    joint_pose.jVal[1] = (req.pose[1]);
    joint_pose.jVal[2] = (req.pose[2]);
    joint_pose.jVal[3] = (req.pose[3]);
    joint_pose.jVal[4] = (req.pose[4]);
    joint_pose.jVal[5] = (req.pose[5]);

    double speed = (double)req.mvvelo;
    double accel = (double)req.mvacc;

    OptionalCond *p = nullptr;

    pthread_mutex_lock(&mutex);

    int sdk_res = robot.joint_move(&joint_pose, MoveMode::ABS, req.is_block, speed, accel, 0.2, p);

    pthread_mutex_unlock(&mutex);

    switch (sdk_res)
    {
    case 0:
        res.ret = 1;
        res.message = "cmd has been executed!";
        break;
    default:
        res.ret = sdk_res;
        res.message = mapErr[sdk_res] + "\n";
        break;
    }

    return true;
}

// 1.3 service jog move -
bool move_jog_callback(robot_msgs::Move::Request &req,
                       robot_msgs::Move::Response &res)
{
    // 1 赋初始值
    double move_velocity = 0;
    CoordType coord_type = COORD_JOINT;

    // 2 选择index   mapping the index and velocity
    // req.index 如果是关节空间   就是  0+,0-,1+,1-,2+,2-,3+,3-,4+,4-,5+,5-
    // req.index 如果是笛卡尔空间 就是x+,x-,y+,y-,z+,z-,rx+,rx-,ry+,ry-,rz+,rz-
    float index_f = static_cast<float>(req.index) / 2 + 0.1;
    int index = static_cast<int>(index_f);

    // 3 选择坐标
    switch (req.coord_mode)
    {
    case 0:
        coord_type = COORD_JOINT;       // 关节坐标 关节空间
        move_velocity = PI / 180.0 * 2; // 关节运动速度 (可修改)
        break;
    case 1:
        coord_type = COORD_BASE; // 基坐标 笛卡尔空间
        if (index >= 3)
        {
            move_velocity = PI / 180.0 * 2; // rx,ry,rz，旋转运动速度 (可修改)
        }
        else
        {
            move_velocity = 2; // 沿x或y或z轴方向运动速度  (可修改)
        }

        break;
    default:
        coord_type = COORD_TOOL; // TCP末端坐标 笛卡尔空间
        if (index >= 3)
        {
            move_velocity = PI / 180.0 * 2; // rx,ry,rz，旋转运动速度 (可修改)
        }
        else
        {
            move_velocity = 2; // 沿x或y或z轴方向运动速度  (可修改)
        }
    }
    // 4 确定速度方向 //判断速度是正方向还是负方向
    if (req.index & 1)
    {
        move_velocity = -move_velocity;
    }
    // 5 进行jog运动
    //  cout << index << ";" << coord_type << ";" << move_velocity << ";"<< pos_cmd << endl;
    if (index_initial != req.index)
    {
        int jog_stop_res = robot.jog_stop(-1);
        if (jog_stop_res == 0)
        {
            sleep(0.2);
            int sdk_res = robot.jog(index, CONTINUE, coord_type, move_velocity, 0);
            switch (sdk_res)
            {
            case 0:
                res.ret = 1;
                res.message = "Position is reached";
                break;
            default:
                res.ret = sdk_res;
                res.message = mapErr[sdk_res] + "\n";
                break;
            }
        }
        else
        {
            res.ret = jog_stop_res;
            res.message = mapErr[jog_stop_res] + "\n";
        }
        index_initial = req.index;
    }
    else
    {
        res.ret = 1;
        res.message = "Moving on";
        ROS_INFO("Moving on");
    }
    return true;
    /* move_jog */
    /* bool move_jog_callback(robot_msgs::Move::Request &req,
            robot_msgs::Move::Response &res)
    {

        //1 赋初始值
        double move_velocity = req.mvvelo;
        CoordType coord_type = COORD_BASE;
        double pos_cmd = 0.0;

        //2 选择index   mapping the index and velocity
        //req.index 如果是关节空间   就是  0+,0-,1+,1-,2+,2-,3+,3-,4+,4-,5+,5-
        //req.index 如果是笛卡尔空间 就是x+,x-,y+,y-,z+,z-,rx+,rx-,ry+,ry-,rz+,rz-
        float index_f = static_cast<float>(req.index) / 2 + 0.1;
        int index = static_cast<int>(index_f);

        //3 选择坐标
        switch (req.coord_mode)
        {
            case 0:
                coord_type = COORD_JOINT; //关节坐标 关节空间
                pos_cmd = PI / 180.0;       //某个关节运动(PI/180.0)弧度 (可修改)
                break;
            case 1:
                coord_type = COORD_BASE;   //基坐标 笛卡尔空间
                if (index >= 3)
                {                         //沿rx,ry,rz，旋转(PI/180.0)弧度 (可修改)
                    pos_cmd = PI / 180.0;
                }
                else
                {
                    pos_cmd = 10;         //沿x或y或z轴方向，运行10mm  (可修改)
                }

                break;
            default:
                coord_type = COORD_TOOL;   //TCP末端坐标 笛卡尔空间
                if (index >= 3)
                {                         //沿rx,ry,rz，旋转(PI/180.0)弧度(可修改)
                    pos_cmd = PI / 180.0;
                }
                else
                {
                    pos_cmd = 10;         //沿x或y或z轴方向，运行10mm  (可修改)
                }
        }
        // 4 确定速度方向 //判断速度是正方向还是负方向
        if(req.index&1)
        {
            move_velocity = -move_velocity;
        }
        //5 进行jog运动
        // cout << index << ";" << coord_type << ";" << move_velocity << ";"<< pos_cmd << endl;
        int jog_stop_res = robot.jog_stop(-1);
        if (jog_stop_res == 0)
        {
            sleep(0.2);
            int sdk_res = robot.jog(index, INCR, coord_type, move_velocity, pos_cmd);
            switch(sdk_res)
            {
                case 0:
                    res.ret = 1;
                    res.message = "Position is reached";
                    break;
                default:
                    res.ret = sdk_res;
                    res.message = mapErr[sdk_res]+"\n";
                    break;
            }
        }
        else
        {
            res.ret = jog_stop_res;
            res.message = mapErr[jog_stop_res]+"\n";
        }
        return true;

    } */
}

// 1.4 service stop move -
bool stop_callback(std_srvs::Empty::Request &req,
                   std_srvs::Empty::Response &res)
{
    servo_mode_open_flag = false;
    // robot.disable_robot();
    pthread_mutex_lock(&mutex);
    int sdk_res = robot.motion_abort();
    pthread_mutex_unlock(&mutex);
    switch (sdk_res)
    {
    case 0:
        ROS_INFO("stop sucess");
        break;
    default:
        cout << "stop error:" << mapErr[sdk_res] << endl;
        break;
    }

    return true;
}

// 1.5 service servo move -
void ServoMovePoseAccepted(const robot_msgs::ServoL::ConstPtr &msg)
{
    // memcpy(expected_pose_servo.data(), &(msg->pose[0]), 6 * 8);
    for (int i = 0; i < 6; i++)
        expected_pose_servo[i] = msg->pose[i];

    if (msg->servo_mode)
        servo_pose_change_flag = true;

    if (!servo_mode_open_flag && msg->servo_mode)
    {
        pthread_mutex_lock(&mutex);

        robot.servo_move_use_joint_LPF(4); // 300 ms
        int tmp = robot.servo_move_enable(true);

        pthread_mutex_unlock(&mutex);

        std::cout << "Servo enable!" << tmp << std::endl;

        sleep(1);

        servo_mode_open_flag = true;
    }
    else if (servo_mode_open_flag && !msg->servo_mode)
    {
        servo_mode_open_flag = false;

        pthread_mutex_lock(&mutex);

        int tmp = robot.servo_move_enable(false);

        pthread_mutex_unlock(&mutex);

        std::cout << "Servo disable!" << tmp << std::endl;

        sleep(1);
    }

    cout << "expected pose:" << expected_pose_servo.transpose() << endl;
    plot_data.data_1 = expected_pose_servo(2);
}

/**
 * @brief    单自由度梯形速度轨迹规划，若距离过短则采用均分规划
 * @param start    起始关节位置
 * @param end      结束关节位置
 * @param start_vel 起始速度
 * @param end_vel 终点速度
 * @param speed_c 巡航速度
 * @param acc_times 加速时间(点数目)
 * @param dec_times 减速时间(点数目)
 * @param total_times 总时间(点数目)
 * @param trajectory 输出轨迹
 */
void Single_Trapezoid_Series(double start, double end, float start_vel, float end_vel,
                             float speed_c, int acc_times, int dec_times, int total_times,
                             std::vector<float> &trajectory)
{
    float path_point;
    float deta;
    float vel;
    int t = 0;
    int dir = 0;

    path_point = start;
    vel = start_vel;
    trajectory.clear();

    dir = (end - start) > 0 ? 1 : -1;

    while (t < total_times) // total times
    {
        if (t < acc_times) // acc times
        {
            vel = vel + (speed_c - start_vel) / acc_times;
        }
        else if (t >= acc_times && t < (total_times - dec_times))
        {
            vel = speed_c;
        }
        else
        {
            vel = vel - (speed_c - end_vel) / dec_times;
        }
        // std::cout << vel << std::endl;

        deta = fabs(end - path_point) > vel ? dir * vel : end - path_point;
        path_point = path_point + deta;

        trajectory.push_back(path_point);

        t++;
    }
    // std::cout << "Trapezoid Series Complete!" << std::endl;
}

/**
 * @brief    单自由度均分规划
 * @param start    起始关节位置
 * @param end      结束关节位置
 * @param total_times 总点数
 * @param trajectory 输出轨迹
 */
void Single_Average_Series(float start, float end, int total_times, std::vector<float> &trajectory)
{
    float path_point;
    float deta;
    int t = 0;

    path_point = start;
    deta = (end - start) / total_times;
    trajectory.clear();

    while (t < total_times)
    {
        path_point = path_point + deta;

        trajectory.push_back(path_point);

        t++;
    }
    // std::cout << "Average Series Complete!" << std::endl;
}

/**
 * @brief 均分规划
 * @param start    起始关节位置
 * @param end      结束关节位置
 * @param total_times 总点数
 * @param trajectory 输出轨迹
 */
void Average_Series(VectorXd start, VectorXd end, double total_times, std::vector<VectorXd> &trajectory)
{
    VectorXd path_point(6);
    VectorXd deta(6);
    int t = 0;
    int num = round(total_times / 0.008);

    path_point = start;
    deta = (end - start) / num;
    trajectory.clear();

    while (t < num)
    {
        path_point = path_point + deta;

        trajectory.push_back(path_point);

        t++;
    }
    // std::cout << "Average Series Complete!" << std::endl;
}

/**
 * @brief 梯形速度轨迹规划，若距离过短则采用均分规划
 * @param joint_start   起始关节位置
 * @param joint_end     结束关节位置
 * @param trajectory    输出轨迹序列
 * @param vel_lim       关节速度限制
 * @param acc_lim       关节加速度限制
 * @param start_vel     起始速度
 * @param end_vel       终止速度
 */
void Trapezoid_Velocity_Series(JointValue joint_start, JointValue joint_end, std::vector<JointValue> &trajectory)
{
    // trajectory.clear();
    // Joint_Value length, path_point;
    // Joint_Flag joint_trapezoid;

    // // Length of acceleration and deceleration
    // Joint_Value acc_length, dec_length;
    // int longest_index = 0;
    // float time;
    // int Times;
    // int Acc_Times, Dec_Times;
    // float len = 0;

    // std::vector<float> path_series[6];

    // // Find the longest joint
    // for (int i = 0; i < 6; i++)
    // {
    //     // Caculate the length of acceleration and deceleration
    //     acc_length[i] = (vel_lim[i] * vel_lim[i] - start_vel[i] * start_vel[i]) / (2 * acc_lim[i]);
    //     dec_length[i] = (vel_lim[i] * vel_lim[i] - end_vel[i] * end_vel[i]) / (2 * acc_lim[i]);

    //     length[i] = fabs(joint_end.jVal[i] - joint_start.jVal[i]);

    //     if (length[i] > len)
    //     {
    //         longest_index = i;
    //         len = length[i];
    //     }

    //     if (length[i] < acc_length[i] + dec_length[i])
    //     {
    //         joint_trapezoid[i] = 0;
    //     }
    //     else
    //     {
    //         joint_trapezoid[i] = 1;
    //     }
    //     // std::cout << " " << i << " " << joint_trapezoid[i] << " " ;
    //     // std::cout << " l " << length[i] << " a " << acc_length[i] << " d " << dec_length[i] << std::endl;
    // }

    // // Caculate times if the longest is Trapezoid
    // if (joint_trapezoid[longest_index] == 1)
    // { /*
    //      time = (vel_lim[longest_index] - start_vel[longest_index]) / acc_lim[longest_index] +
    //              (vel_lim[longest_index] - end_vel[longest_index]) / acc_lim[longest_index] +
    //              (length[longest_index] - acc_length[longest_index] - dec_length[longest_index]) / vel_lim[longest_index]; */

    //     Acc_Times = static_cast<int>((vel_lim[longest_index] - start_vel[longest_index]) / acc_lim[longest_index] + 1);
    //     Dec_Times = static_cast<int>((vel_lim[longest_index] - end_vel[longest_index]) / acc_lim[longest_index] + 1);
    //     // Acc_Times = static_cast<int>((vel_lim[longest_index] - start_vel[longest_index]) / acc_lim[longest_index]);
    //     // Dec_Times = static_cast<int>((vel_lim[longest_index] - end_vel[longest_index]) / acc_lim[longest_index]);

    //     time = Acc_Times + Dec_Times +
    //            (length[longest_index] - acc_length[longest_index] - dec_length[longest_index]) / vel_lim[longest_index];

    //     // std::cout << " " << i << " " << joint_trapezoid[i] << " " ;
    //     // std::cout << " t " << time << " a " << Acc_Times << " d " << Dec_Times << std::endl;
    // }
    // else
    // {
    //     /* time = (vel_lim[longest_index] - start_vel[longest_index]) / acc_lim[longest_index] +
    //             (vel_lim[longest_index] - end_vel[longest_index]) / acc_lim[longest_index]; */

    //     time = 2 * length[longest_index] / (start_vel[longest_index] + end_vel[longest_index]);
    // }

    // // Times of trajectory
    // // Times = static_cast<int>(time + 1);
    // Times = static_cast<int>(time);

    // // std::cout << " " << Times << std::endl;

    // // Planning the trajectory
    // for (int i = 0; i < 6; i++)
    // {
    //     if (joint_trapezoid[i] == 1)
    //     {

    //         /* if(i == longest_index)
    //         {
    //             Single_Trapezoid_Series(joint_start.jVal[i], joint_end.jVal[i], start_vel[i], end_vel[i], vel_lim[i], Acc_Times, Dec_Times, Times, path_series[i]);
    //         }
    //         else
    //         { */
    //         float cruise_vel;

    //         cruise_vel = fabs((2 * fabs(joint_end.jVal[i] - joint_start.jVal[i]) - start_vel[i] * Acc_Times - end_vel[i] * Dec_Times) / (2 * Times - Acc_Times - Dec_Times));

    //         Single_Trapezoid_Series(joint_start.jVal[i], joint_end.jVal[i], start_vel[i], end_vel[i], cruise_vel, Acc_Times, Dec_Times, Times, path_series[i]);
    //         //}
    //     }
    //     else
    //     {
    //         Single_Average_Series(joint_start.jVal[i], joint_end.jVal[i], Times, path_series[i]);
    //     }
    // }

    // for (int i = 0; i < path_series[longest_index].size(); i++)
    // {
    //     JointValue joint_data;
    //     // joint_data.clear();
    //     for (int j = 0; j < 6; j++)
    //     {
    //         joint_data.jVal[j] = path_series[j][i];
    //     }

    //     trajectory.push_back(joint_data);
    // }

    // // std::cout << "Planning Complete!" << std::endl;
}

/**
 * @brief 伺服关节运动,使用给定序列
 * @param trajectory 轨迹
 * @param start      起始点列
 * @param end        终止点列
 */
// bool servo_move_joint(vector<VectorXd> &trajectory, int start, int end)
// {
//     JointValue tmp_expected_joint_servo;
//
//     robot.servo_move_use_joint_LPF(4);
//     robot.servo_move_enable(true);
//
//     std::cout << "Servo enable!" << std::endl;
//
//     usleep(8 * 1000);
//     for (int t = start; t < end; t++)
//     {
//         memcpy(tmp_expected_joint_servo.jVal, trajectory[t].data(), 6 * 8);
//         int sdk_res = robot.servo_j(&tmp_expected_joint_servo, ABS, 4);
//         if (sdk_res != 0)
//         {
//             cout << "servo error:" << mapErr[sdk_res] << endl;
//             return false;
//         }
//     }
//     usleep(8 * 1000);
//
//     robot.servo_move_enable(false);
//
//     return true;
// }

#pragma region /* Robot Setting Server */
// 3.1 service set user frame -
bool set_user_frame_callback(robot_msgs::SetUserFrame::Request &req,
                             robot_msgs::SetUserFrame::Response &res)
{
    CartesianPose cart;
    cart.tran.x = req.pose[0] * 1000;
    cart.tran.y = req.pose[1] * 1000;
    cart.tran.z = req.pose[2] * 1000;
    cart.rpy.rx = req.pose[3];
    cart.rpy.ry = req.pose[4];
    cart.rpy.rz = req.pose[5];

    int id = req.user_num;

    res.ret = robot.set_user_frame_data(id, &cart, "Base Coord");

    switch (res.ret)
    {
    case 0:
        res.message = "User frame is set";
        break;
    default:
        res.message = "Failed to set user frame";
        break;
    }

    return true;
}

// 3.2 service set tcp -
bool set_tcp(robot_msgs::SetTcp::Request &req,
             robot_msgs::SetTcp::Response &res)
{
    CartesianPose cart;
    cart.tran.x = req.pose[0] * 1000;
    cart.tran.y = req.pose[1] * 1000;
    cart.tran.z = req.pose[2] * 1000;
    cart.rpy.rx = req.pose[3];
    cart.rpy.ry = req.pose[4];
    cart.rpy.rz = req.pose[5];

    // int id = req.tool_num;
    int id = 1;
    cout << "pose_tcp" << endl;
    cout << cart.tran.x << endl;
    cout << cart.tran.y << endl;
    cout << cart.tran.z << endl;
    cout << cart.rpy.rx << endl;
    cout << cart.rpy.ry << endl;
    cout << cart.rpy.rz << endl;

    int sdk_res_data = robot.set_tool_data(id, &cart, "Tool Coord"); // 接受sdk返回值
    int sdk_res_id = robot.set_tool_id(id);                          // 设置id

    // 转换为需求字典

    if (sdk_res_data == 0)
    {
        res.ret = 1;
        res.message = "Tcp is set";
    }
    else
    {
        res.message = mapErr[sdk_res_data];
    }

    if (sdk_res_id == 0)
    {
        res.ret = 1;
        res.message = "Tcp is set";
    }
    else
    {
        res.message = mapErr[sdk_res_id];
    }

    return true;
}

// 3.3 service enable teach drag -
bool enable_teach_drag_callback(std_srvs::SetBool::Request &req,
                                std_srvs::SetBool::Response &res)
{
    bool enable_teach_drag = req.data;

    res.success = !robot.drag_mode_enable(enable_teach_drag);

    if (res.success)
    {
        res.message = "OK";
    }
    else
    {
        res.message = "Fail";
    }

    return true;
}

// 3.4 service enable servo -
bool enable_servo_callback(std_srvs::SetBool::Request &req,
                           std_srvs::SetBool::Response &res)
{
    bool enable_servo = req.data;

    res.success = !robot.servo_move_enable(enable_servo);

    if (res.success)
    {
        res.message = "OK";
    }
    else
    {
        res.message = "Fail";
    }

    return true;
}

// 3.5 service set payload -
bool set_payload_callback(robot_msgs::SetLoad::Request &req,
                          robot_msgs::SetLoad::Response &res)
{
    PayLoad payload;
    int tool_id = req.tool_num;

    payload.centroid.x = req.xc * 1000;
    payload.centroid.y = req.yc * 1000;
    payload.centroid.z = req.zc * 1000;
    payload.mass = req.mass;

    robot.set_tool_id(tool_id);
    int sdk_res = robot.set_payload(&payload);

    switch (sdk_res)
    {
    case 0:
        res.message = "Payload is set";
        res.ret = 1;
        break;
    default:
        res.ret = sdk_res;
        res.message = mapErr[sdk_res];
        break;
    }

    return true;
}

// 3.6 service clear error -
bool clear_err_callback(robot_msgs::ClearErr::Request &req,
                        robot_msgs::ClearErr::Response &res)
{
    RobotState state;
    RobotStatus status;

    robot.get_robot_state(&state);
    robot.get_robot_status(&status);

    // estoped
    // estoped == 0 enable
    ROS_INFO("status.enabled:%d", status.enabled);

    if (!state.estoped)
    {
        res.ret = robot.collision_recover();
        switch (res.ret)
        {
        case 0:
            res.message = "Collision error is clear\n";
            break;
        default:
            // res.message = "Failed to clear error\n";
            res.message = mapErr[res.ret] + "\n";
            break;
        }

        // Robot re-enable
        if (!status.enabled)
        {
            if (robot.power_on())
                ROS_INFO("power_on error!!");
            res.ret = robot.enable_robot();
            switch (res.ret)
            {
            case 0:
                res.message = "Robot is recovered and enabled";
                break;
            default:
                // res.message = "Failed to enable robot";
                res.message = mapErr[res.ret] + "\n";
                break;
            }
        }
        // Robot collision recover
    }
    // res.ret = robot.enable_robot();
    // estoped == 1 un_enable
    return true;
}

// 3.7 service set collision level -
bool set_collision_callback(robot_msgs::SetCollision::Request &req,
                            robot_msgs::SetCollision::Response &res)
{
    int collision_level = 0;
    std::string msg = "Collision level ";

    if (req.is_enable == 0)
    {
        collision_level = 0;
    }
    else
    {
        if (req.value <= 25)
        {
            collision_level = 1;
        }
        else if (req.value <= 50)
        {
            collision_level = 2;
        }
        else if (req.value <= 75)
        {
            collision_level = 3;
        }
        else if (req.value <= 100)
        {
            collision_level = 4;
        }
        else
        {
            collision_level = 5;
        }
    }
    res.ret = robot.set_collision_level(collision_level);

    switch (res.ret)
    {
    case 0:

        msg = msg + to_string(collision_level) + " is set";
        res.message = msg;
        break;
    default:
        res.message = mapErr[res.ret] + "\n";
        break;
    }

    return true;
}
#pragma endregion /* Robot Setting Server */

/* enable or disable robot */
void robot_state_control_callback()
{
    bool enable_robot, disable_robot; // enable or disable robot
    ros::param::get("/enable_robot", enable_robot);
    ros::param::get("/disable_robot", disable_robot);

    if (enable_robot)
    {
        robot.power_on();
        robot.enable_robot();
        ros::param::set("/enable_robot", false);
    }

    if (disable_robot)
    {
        robot.disable_robot();
        // robot.power_off();
        ros::param::set("/disable_robot", false);
    }
}

void *PlotData(void *args)
{
    ros::NodeHandle *n = (ros::NodeHandle *)args;

    ros::Publisher plot_pub = n->advertise<admittance_control::Plot>("/plot_data_2", 100);

    ros::Rate rate(500);
    while (ros::ok())
    {
        plot_pub.publish(plot_data);

        rate.sleep();
    }
}

bool GetPositionCallback(robot_msgs::GetPosition::Request &req,
                         robot_msgs::GetPosition::Response &res)
{
    if (req.is_tcp_position)
    {
        CartesianPose tool_pose;

        pthread_mutex_lock(&mutex);

        robot.get_tcp_position(&tool_pose);

        pthread_mutex_unlock(&mutex);

        res.position[0] = tool_pose.tran.x / 1000;
        res.position[1] = tool_pose.tran.y / 1000;
        res.position[2] = tool_pose.tran.z / 1000;
        res.position[3] = tool_pose.rpy.rx;
        res.position[4] = tool_pose.rpy.ry;
        res.position[5] = tool_pose.rpy.rz;
    }
    else
    {
        JointValue joint_pose;

        pthread_mutex_lock(&mutex);

        robot.get_joint_position(&joint_pose);

        pthread_mutex_unlock(&mutex);

        memcpy(&(res.position[0]), joint_pose.jVal, 6 * 8);
    }

    return true;
}

void *RobotStatePublish(void *args)
{
    ros::NodeHandle *n = (ros::NodeHandle *)args;

    // 2.1 robot tcp publisher -
    ros::Publisher tool_point_pub = n->advertise<geometry_msgs::TwistStamped>("/robot_driver/tool_point", 1);

    // 2.2 robot joint angle publisher -
    ros::Publisher joint_states_pub = n->advertise<sensor_msgs::JointState>("/robot_driver/joint_states", 1);

    // 2.3 robot state publisher -
    // ros::Publisher robot_state_pub = n.advertise<robot_msgs::RobotMsg>("/robot_driver/robot_states", 10);

    ros::Rate rate(100);
    while (ros::ok())
    {
        geometry_msgs::TwistStamped tool_point;

        sensor_msgs::JointState joint_states;

        RobotStatus robot_status;

        pthread_mutex_lock(&mutex);

        robot.get_robot_status(&robot_status);

        pthread_mutex_unlock(&mutex);

        if (!robot_status.is_socket_connect)
        {
            ROS_ERROR("connect error!!!");
            continue;
        }

        tool_point.twist.linear.x = robot_status.cartesiantran_position[0] / 1000;
        tool_point.twist.linear.y = robot_status.cartesiantran_position[1] / 1000;
        tool_point.twist.linear.z = robot_status.cartesiantran_position[2] / 1000;

        tool_point.twist.angular.x = robot_status.cartesiantran_position[3];
        tool_point.twist.angular.y = robot_status.cartesiantran_position[4];
        tool_point.twist.angular.z = robot_status.cartesiantran_position[5];

        for (int i = 0; i < 6; i++)
        {
            joint_states.position.push_back(robot_status.joint_position[i]); // write data into standard ros msg
            joint_states.name.push_back("joint_" + std::to_string(i + 1));
            joint_states.header.stamp = ros::Time::now();
        }

        tool_point_pub.publish(tool_point);
        joint_states_pub.publish(joint_states); // publish data

        memcpy(current_joint_servo.data(), robot_status.joint_position, 6 * 8);

        plot_data.data_2 = tool_point.twist.linear.z;
        plot_data.data_4 = robot_status.joint_position[1];
        plot_data.data_5 = robot_status.joint_position[2];

        rate.sleep();

#pragma region // 2.3 robot state publish
// robot_msgs::RobotMsg robot_state;

// RobotState robotstate;
// RobotStatus robotstatus;
// ProgramState programstate;
// BOOL in_pos = true;
// BOOL in_col = false;

// if (robot.is_in_pos(&in_pos))
//     ROS_INFO("Failed to get robot pos!");
// if (robot.get_program_state(&programstate))
//     ROS_INFO("Failed to get program state!");
// if (robot.get_robot_status(&robotstatus))
//     ROS_INFO("Failed to get robot status!");
// if (robot.get_robot_state(&robotstate))
//     ROS_INFO("Failed to get robot state!");
// if (robot.is_in_collision(&in_col))
//     ROS_INFO("Failed to get robot collision state!");

// if (robotstate.estoped)
// {
//     robot_state.state = 2;
//     // ROS_INFO("Robot program is running!");
// }
// else if (robotstatus.errcode != 0)
// {
//     robot_state.state = 4;
//     // ROS_INFO("Robot is in error!")
// }
// else if (in_pos && programstate == PROGRAM_IDLE && robotstatus.drag_status == 0)
// {
//     robot_state.state = 0;
//     // ROS_INFO("Robot is in pos!");
// }
// else if (programstate == PROGRAM_PAUSED)
// {
//     robot_state.state = 1;
//     // ROS_INFO("Robot program is paused!");
// }
// else if (!in_pos || programstate == PROGRAM_RUNNING || robotstatus.drag_status == 1)
// {
//     robot_state.state = 3;
//     // ROS_INFO("Robot program is running!");
// }

// /*
// if(in_pos && programstate == PROGRAM_IDLE)
// {
//     robot_state.state = 0;
//     //ROS_INFO("Robot is in pos!");
// }
// else if(programstate == PROGRAM_PAUSED)
// {
//     robot_state.state = 1;
//     //ROS_INFO("Robot program is paused!");
// }
// else if(robotstate.estoped)
// {
//     robot_state.state = 2;
//     //ROS_INFO("Robot is emergency stopped!");
// }
// else if(!in_pos || programstate == PROGRAM_RUNNING)
// {
//     robot_state.state = 3;
//     //ROS_INFO("Robot program is running!");
// }
// else if(robotstatus.errcode != 0)
// {
//     robot_state.state = 4;
//     //ROS_INFO("Robot is in error!");
// }
// */

// robot_state.mode = 2;
// // ROS_INFO("Robot is in distance mode!");

// if (robotstate.poweredOn)
// {
//     robot_state.motor_sync = 1;
//     // ROS_INFO("Robot is in synchronization!");
// }
// else
// {
//     robot_state.motor_sync = 0;
//     // ROS_INFO("Robot is in unsynchronization!");
// }

// if (robotstatus.enabled)
// {
//     // 客户文档要求servo_enable 代表机器人是否使能，sdk内的servo_enable表示透传模式下是否使能
//     robot_state.servo_enable = 1;
//     // ROS_INFO("Servo mode is enable!");
// }
// else
// {
//     robot_state.servo_enable = 0;
//     // ROS_INFO("Servo mode is disable!");
// }

// if (in_col)
// {
//     robot_state.collision_state = 1;
//     // ROS_INFO("Robot is in collision!");
// }
// else
// {
//     robot_state.collision_state = 0;
//     // ROS_INFO("Robot is not in collision!");
// }
// // robot_state.state = 0;
// // robot_state.mode = 2;
// // robot_state.servo_enable = 1;
// // robot_state.motor_sync = 1;
// // robot_state.collision_state = 0;

// robot_state_pub.publish(robot_state);
#pragma endregion /* Robot State Publisher */
    }
}

void *ServoMove(void *args)
{
    ros::NodeHandle n_servo;

    ros::CallbackQueue servo_queue;
    n_servo.setCallbackQueue(&servo_queue);

    // 1.5 service servo move -
    ros::Subscriber servo_move_sub = n_servo.subscribe("/robot_driver/servo_move", 1, &ServoMovePoseAccepted);

    while (ros::ok())
    {
        servo_queue.callOne(ros::WallDuration(1.0));
        if (servo_mode_open_flag && servo_pose_change_flag)
        {
            vector<VectorXd> tra;
            VectorXd expected_joint_servo(6);
            JointValue tmp_expected_joint_servo;
            JointValue tmp_current_joint_servo;
            CartesianPose tmp_expected_pose_servo;

            expected_joint_servo = VectorXd::Zero(6);

            expected_pose_servo.block<3, 1>(0, 0) = expected_pose_servo.block<3, 1>(0, 0) * 1000;

            memcpy(tmp_expected_joint_servo.jVal, expected_joint_servo.data(), 6 * 8);
            memcpy(tmp_current_joint_servo.jVal, current_joint_servo.data(), 6 * 8);
            memcpy(&(tmp_expected_pose_servo.tran.x), expected_pose_servo.data(), 6 * 8);

            pthread_mutex_lock(&mutex);

            robot.kine_inverse(&tmp_current_joint_servo, &tmp_expected_pose_servo, &tmp_expected_joint_servo);

            pthread_mutex_unlock(&mutex);

            memcpy(expected_joint_servo.data(), tmp_expected_joint_servo.jVal, 6 * 8);

            Average_Series(current_joint_servo, expected_joint_servo, 0.1, tra);

            // Trapezoid_Velocity_Series(expected_joint_servo, tra, vel_lim, acc_lim, start_vel, end_vel);

            servo_pose_change_flag = false;

            for (int t = 0; t < tra.size(); t++)
            {
                servo_queue.callOne(ros::WallDuration(0));
                if (servo_pose_change_flag)
                {
                    expected_pose_servo.block<3, 1>(0, 0) = expected_pose_servo.block<3, 1>(0, 0) * 1000;

                    memcpy(tmp_expected_joint_servo.jVal, expected_joint_servo.data(), 6 * 8);
                    memcpy(tmp_current_joint_servo.jVal, current_joint_servo.data(), 6 * 8);
                    memcpy(&(tmp_expected_pose_servo.tran.x), expected_pose_servo.data(), 6 * 8);

                    pthread_mutex_lock(&mutex);

                    robot.kine_inverse(&tmp_current_joint_servo, &tmp_expected_pose_servo, &tmp_expected_joint_servo);

                    pthread_mutex_unlock(&mutex);

                    memcpy(expected_joint_servo.data(), tmp_expected_joint_servo.jVal, 6 * 8);

                    Average_Series(current_joint_servo, expected_joint_servo, 0.1, tra);

                    // Trapezoid_Velocity_Series(expected_joint_servo, tra, vel_lim, acc_lim, start_vel, end_vel);

                    servo_pose_change_flag = false;
                    t = 0;
                    cout << "change expected pose" << expected_pose_servo.transpose() << endl;
                }

                memcpy(tmp_expected_joint_servo.jVal, tra[t].data(), 6 * 8);
                cout << "路径点" << t << ": " << tra[t].transpose() << endl;
                plot_data.data_6 = tmp_expected_joint_servo.jVal[1];
                plot_data.data_7 = tmp_expected_joint_servo.jVal[2];

                pthread_mutex_lock(&mutex);

                int sdk_res = robot.servo_j(&tmp_expected_joint_servo, ABS, 1);

                pthread_mutex_unlock(&mutex);

                if (sdk_res != 0)
                {
                    cout << "servo error:" << mapErr[sdk_res] << endl;
                    int res = robot.motion_abort();
                    switch (res)
                    {
                    case 0:
                        ROS_INFO("stop sucess");
                        break;
                    default:
                        cout << "stop error:" << mapErr[res] << endl;
                        break;
                    }
                }
            }
            usleep(8 * 1000);
        }
    }
}

int main(int argc, char **argv)
{
    RobotStatus ret_status;

    ros::init(argc, argv, "connect_robot");

    ros::NodeHandle n;

    // init params
    string ip = "192.168.50.170";
    expected_pose_servo = VectorXd::Zero(6);
    pthread_mutex_init(&mutex, NULL);

    ros::param::set("/enable_robot", false);
    ros::param::set("/disable_robot", false);
    ros::param::set("robot_ip", ip);

    /* services and topics */

    // 1.1 service move line -
    ros::ServiceServer service_movel = n.advertiseService("/robot_driver/move_line", movel_callback);

    // 1.2 service move joint -
    ros::ServiceServer service_movej = n.advertiseService("/robot_driver/move_joint", movej_callback);

    // 1.3 service jog move -
    // ros::ServiceServer move_jog = n.advertiseService("/robot_driver/move_jog", move_jog_callback);

    // 1.4 service stop move -
    ros::ServiceServer service_stop = n.advertiseService("/robot_driver/stop_move", stop_callback);

    // 2.4 service get robot position -
    ros::ServiceServer service_position = n.advertiseService("/robot_driver/update_position", GetPositionCallback);

    // 3.1 service set user frame -
    // ros::ServiceServer service_set_user_frame = n.advertiseService("/robot_driver/set_user_frame", set_user_frame_callback);

    // 3.2 service set tcp -
    // ros::ServiceServer service_set_tcp = n.advertiseService("/robot_driver/set_tcp", set_tcp);

    // 3.3 service enable teach drag -
    // ros::ServiceServer service_enable_teach_drag = n.advertiseService("/robot_driver/teach_drag", enable_teach_drag_callback);

    // 3.4 service enable servo -
    // ros::ServiceServer service_enable_servo = n.advertiseService("/robot_driver/servo_ctr", enable_servo_callback);

    // 3.5 service set payload -
    // ros::ServiceServer service_set_payload = n.advertiseService("/robot_driver/set_payload", set_payload_callback);

    // 3.6 service clear error -
    // ros::ServiceServer service_clear_err = n.advertiseService("/robot_driver/clear_err", clear_err_callback);

    // 3.7 service set collision level -
    // ros::ServiceServer service_set_collision = n.advertiseService("/robot_driver/set_collision", set_collision_callback);

    ROS_INFO("Try to connect robot");

    // connect robot
    std::cout << "-----------" << endl;
    robot.login_in(ip.c_str());
    sleep(1);
    robot.power_on();
    sleep(1);
    robot.enable_robot();
    robot.get_robot_status(&ret_status);
    if (ret_status.enabled == false)
        ROS_INFO("Robot:%s failed", ip.c_str());

    ROS_INFO("Robot:%s enable", ip.c_str());
    ros::Duration(1).sleep();
    ROS_INFO("Robot:%s ready!", ip.c_str());

    // pthread_t tids_1;
    // pthread_create(&tids_1, NULL, PlotData, &n);

    pthread_t tids_2;
    pthread_create(&tids_2, NULL, RobotStatePublish, &n);

    pthread_t tids_3;
    pthread_create(&tids_3, NULL, ServoMove, &n);

    // ros::MultiThreadedSpinner s(4);S
    ros::spin();

    // pthread_join(tids_1, NULL);
    pthread_join(tids_2, NULL);
    pthread_join(tids_3, NULL);
    std::cout << "shut down" << std::endl;

    pthread_mutex_lock(&mutex);
    int ret = robot.login_out();
    cout << ret << endl;
    pthread_mutex_unlock(&mutex);

    pthread_mutex_destroy(&mutex);
    return 0;
}
