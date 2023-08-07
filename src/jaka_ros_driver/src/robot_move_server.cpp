#include "ros/ros.h"
#include "std_msgs/String.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "robot_msgs/Move.h"
#include "robot_msgs/RobotMsg.h"
#include "robot_msgs/ServoL.h"

#include "std_srvs/Empty.h"
#include "std_srvs/SetBool.h"
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
#include "libs/jktypes.h"

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
VectorXd expected_pose_servo(6);
VectorXd current_joint_servo(6);
bool servo_mode = false;

// 1.1 service move line -
bool movel_callback(robot_msgs::Move::Request &req,
                    robot_msgs::Move::Response &res)
{
    servo_mode = false;

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

    int sdk_res = robot.linear_move(&cart, MoveMode::ABS, req.is_block, speed, accel, 0.3, p);

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
    servo_mode = false;

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

    int sdk_res = robot.joint_move(&joint_pose, MoveMode::ABS, req.is_block, speed, accel, 0.2, p);

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
    servo_mode = false;

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
    servo_mode = false;

    // robot.disable_robot();
    int sdk_res = robot.motion_abort();
    switch (sdk_res)
    {
    case 0:
        ROS_INFO("stop sucess");
        break;
    default:
        cout << "error:" << mapErr[sdk_res] << endl;
        break;
    }

    return true;
}

// 1.5 service servo move -
void ServoMovePoseAccepted(const robot_msgs::ServoL::ConstPtr &msg)
{
    memcpy(expected_pose_servo.data(), &(msg->pose[0]), 6 * 8);
    servo_mode = msg->servo_mode;
    cout << expected_pose_servo.transpose() << endl;
}

void JointStatesRecord(const sensor_msgs::JointState::ConstPtr &msg)
{
    current_joint_servo << msg->position[0], msg->position[1], msg->position[2],
        msg->position[3], msg->position[4], msg->position[5];
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
bool servo_move_joint(vector<VectorXd> &trajectory, int start, int end)
{
    JointValue tmp_expected_joint_servo;

    robot.servo_move_use_joint_LPF(4);
    robot.servo_move_enable(true);

    std::cout << "Servo enable!" << std::endl;

    usleep(8 * 1000);
    for (int t = start; t < end; t++)
    {
        memcpy(tmp_expected_joint_servo.jVal, trajectory[t].data(), 6 * 8);
        int sdk_res = robot.servo_j(&tmp_expected_joint_servo, ABS, 4);
        if (sdk_res != 0)
        {
            cout << "error:" << mapErr[sdk_res] << endl;
            return false;
        }
    }
    usleep(8 * 1000);

    robot.servo_move_enable(false);

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_move_server");

    ros::NodeHandle n;

    // 1.1 service move line -
    // ros::ServiceServer service_movel = n.advertiseService("/robot_driver/move_line", movel_callback);

    // 1.2 service move joint -
    ros::ServiceServer service_movej = n.advertiseService("/robot_driver/move_joint", movej_callback);

    // 1.3 service jog move -
    // ros::ServiceServer move_jog = n.advertiseService("/robot_driver/move_jog", move_jog_callback);

    // 1.4 service stop move -
    ros::ServiceServer service_stop = n.advertiseService("/robot_driver/stop_move", stop_callback);

    // 1.5 service servo move -
    ros::Subscriber servo_move_sub = n.subscribe("/robot_driver/servo_move", 10, ServoMovePoseAccepted);
    ros::Subscriber tool_point_sub = n.subscribe("/robot_driver/joint_states", 1, JointStatesRecord);

    vector<VectorXd> tra;
    VectorXd expected_joint_servo(6);
    JointValue tmp_expected_joint_servo;
    JointValue tmp_current_joint_servo;
    CartesianPose tmp_expected_pose_servo;

    expected_pose_servo = VectorXd::Zero(6);
    expected_joint_servo = VectorXd::Zero(6);

    while (ros::ok())
    {
        ros::spinOnce();

        if (servo_mode)
        {
            memcpy(tmp_expected_joint_servo.jVal, expected_joint_servo.data(), 6 * 8);
            memcpy(tmp_current_joint_servo.jVal, current_joint_servo.data(), 6 * 8);
            memcpy(&(tmp_expected_pose_servo.tran.x), expected_pose_servo.data(), 6 * 8);

            robot.kine_inverse(&tmp_expected_joint_servo, &tmp_expected_pose_servo, &tmp_current_joint_servo);

            memcpy(expected_joint_servo.data(), tmp_expected_joint_servo.jVal, 6 * 8);

            Average_Series(current_joint_servo, expected_joint_servo, 0.1, tra);

            // Trapezoid_Velocity_Series(expected_joint_servo, tra, vel_lim, acc_lim, start_vel, end_vel);

            // bool code = servo_move_joint(tra, 0, tra.size());

            // servo_mode = false;

            // if (!code)
            // {
            //     int sdk_res = robot.motion_abort();
            //     switch (sdk_res)
            //     {
            //     case 0:
            //         ROS_INFO("stop sucess");
            //         break;
            //     default:
            //         cout << "error:" << mapErr[sdk_res] << endl;
            //         break;
            //     }
            //     return 0;
            // }
        }
    }

    return 0;
}