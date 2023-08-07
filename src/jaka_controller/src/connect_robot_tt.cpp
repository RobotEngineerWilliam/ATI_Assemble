//
// Created by zhao on 2023/8/7.
//

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Twist.h"
#include <pthread.h>
#include <unistd.h>
#include <sstream>

#include <vector>

#include "libs/robot.h"
#include "libs/conversion.h"
#include <map>
#include <string>

#include "jaka_controller/Multi_Move.h"
#include "jaka_controller/trapezoid_plan.h"

using std::cout;
using std::endl;
using std::string;
using std::vector;

/* 伺服模式所需变量 */
std::vector<JointValue> tra;
typedef float Joint_Value[6];
typedef int Joint_Flag[6];

/* global variables */
JAKAZuRobot robot;
bool servo_mode = true;

RobotStatus status;
static pthread_mutex_t tool_point_mutex;

const double PI = 3.1415926;

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

void *tool_point_tread_func(void *args)
{
    ros::NodeHandle *n = (ros::NodeHandle *)args;

    geometry_msgs::Twist tool_point;
    //        CartesianPose tool_pose;
    sensor_msgs::JointState joint_states;
    joint_states.position.resize(6);
    joint_states.name.resize(6);

    ros::Publisher tcp_pub = n->advertise<geometry_msgs::Twist>("/robot_driver/tool_point", 1);
    ros::Publisher joint_states_pub = n->advertise<sensor_msgs::JointState>("/robot_driver/joint_states", 1);

    while (ros::ok())
    {
        pthread_mutex_lock(&tool_point_mutex);
        // robot.get_tcp_position(&tool_pose);
        robot.get_robot_status(&status);
        pthread_mutex_unlock(&tool_point_mutex);

        tool_point.linear.x = status.cartesiantran_position[0] / 1000;
        tool_point.linear.y = status.cartesiantran_position[1] / 1000;
        tool_point.linear.z = status.cartesiantran_position[2] / 1000;

        // rpy
        tool_point.angular.x = status.cartesiantran_position[3];
        tool_point.angular.y = status.cartesiantran_position[4];
        tool_point.angular.z = status.cartesiantran_position[5];

        for (int i = 0; i < 6; i++)
        {
            joint_states.position[i] = status.joint_position[i]; // write data into standard ros msg
            int j = i + 1;
            joint_states.name[i] = "joint_" + std::to_string(j);
        }
        joint_states.header.stamp = ros::Time::now();
        joint_states_pub.publish(joint_states);
        tcp_pub.publish(tool_point);
        usleep(8 * 1000);
    }
}

void tool_point_pub(ros::NodeHandle &n)
{
    pthread_t tids_2;
    pthread_mutex_init(&tool_point_mutex, NULL);
    pthread_create(&tids_2, NULL, tool_point_tread_func, &n);
}

int sendJointMovePoint(CartesianPose pose, float speed)
{
    JointValue ref_joint_pose;
    JointValue joint_pose;

    robot.get_joint_position(&ref_joint_pose);
    int inverse_code = robot.kine_inverse(&ref_joint_pose, &pose, &joint_pose);
    if (inverse_code == 0)
    {
        // int code = 0;
        for (int i = 0; i < 6; ++i)
        {
            std::cout << joint_pose.jVal[i] << ' ';
        }
        std::cout << '\n';

        int code = robot.joint_move(&joint_pose, MoveMode::ABS, true, speed);
        return code;
    }
    else
    {
        cout << "fail ik" << endl;
        return -1;
    }
}

/**
 * @brief 伺服关节运动,使用给定序列
 * @param trajectory 轨迹
 * @param start      起始点列
 * @param end        终止点列
 */
void servo_move_joint(std::vector<JointValue> &trajectory, int start, int end)
{
    robot.servo_move_use_joint_LPF(4);
    robot.servo_move_enable(true);

    std::cout << "Servo enable!" << std::endl;

    usleep(8 * 1000);
    for (int t = start; t < end; t++)
    {
        robot.servo_j(&trajectory[t], ABS, 4);
    }
    usleep(8 * 1000);

    robot.servo_move_enable(false);
}

bool multi_move_callback(jaka_controller::Multi_Move::Request &req,
                         jaka_controller::Multi_Move::Response &res)
{
    int type = req.type;
    OptionalCond *p = nullptr;
    cout << "Move type: " << type << endl;
    cout << "**********************************************" << endl;

    double speed;
    double accel; // case 1 2 3 4 5

    JointValue joint_pose;     // case 1 4
    JointValue ref_joint_pose; // case 4
    CartesianPose cart;        // case 2 4 10

    CartesianPose end_pose;
    CartesianPose mid_pose; // case 3

    Joint_Value vel_lim, acc_lim;
    Joint_Value start_vel, end_vel; // case 4 5

    // std::cout << req.mvvelo << std::endl;
    // std::cout << req.mvacc << std::endl;

    // speed = (double)req.mvvelo/180*PI;
    // accel = (double)req.mvacc/180*PI;

    switch (type)
    {
    case 1: // 关节角度
        joint_pose.jVal[0] = req.pose[0];
        joint_pose.jVal[1] = req.pose[1];
        joint_pose.jVal[2] = req.pose[2];
        joint_pose.jVal[3] = req.pose[3];
        joint_pose.jVal[4] = req.pose[4];
        joint_pose.jVal[5] = req.pose[5];

        speed = (double)20.0 / 180 * PI;
        accel = (double)10.0 / 180 * PI;

        res.ret = robot.joint_move(&joint_pose, MoveMode::ABS, true, speed, accel, 0.3, p);
        break;

    case 2:
        // meter convert to mm  笛卡尔空间
        cart.tran.x = req.pose[0] * 1000;
        cart.tran.y = req.pose[1] * 1000;
        cart.tran.z = req.pose[2] * 1000;

        cart.rpy.rx = req.pose[3];
        cart.rpy.ry = req.pose[4];
        cart.rpy.rz = req.pose[5];

        cout << cart.tran.x << endl;
        cout << cart.tran.y << endl;
        cout << cart.tran.z << endl;
        cout << cart.rpy.rx << endl;
        cout << cart.rpy.ry << endl;
        cout << cart.rpy.rz << endl;

        speed = (float)8.0 / 180 * PI;
        accel = (float)5.0 / 180 * PI;
        res.ret = sendJointMovePoint(cart, speed);
        break;

    case 4: // serve mode  伺服模式 笛卡尔空间
    {
        // meter convert to mm
        cart.tran.x = req.pose[0] * 1000;
        cart.tran.y = req.pose[1] * 1000;
        cart.tran.z = req.pose[2] * 1000;

        cart.rpy.rx = req.pose[3];
        cart.rpy.ry = req.pose[4];
        cart.rpy.rz = req.pose[5];

        for (int k = 0; k < 6; k++)
        {
            // Fast
            //  vel_lim[k] = 0.01;//0.005
            //  acc_lim[k] = 0.002;//0.0005

            // start_vel[k] = 0.003;//0.002
            // end_vel[k] = 0.007;//0.002
            // faster
            vel_lim[k] = 0.003;  // 0.005
            acc_lim[k] = 0.0005; // 0.0005
            // slow
            // vel_lim[k] = 0.002;//0.005
            // acc_lim[k] = 0.0005;//0.0005

            //  start_vel[k] = 0.002;//0.002
            //  end_vel[k] = 0.002;//0.002

            start_vel[k] = 0.002; // 0.002
            end_vel[k] = 0.002;   // 0.002
        }
        robot.get_joint_position(&ref_joint_pose);
        std::clock_t t1 = std::clock();
        res.ret = robot.kine_inverse(&ref_joint_pose, &cart, &joint_pose);

        Trapezoid_Velocity_Series(ref_joint_pose, joint_pose, tra, vel_lim, acc_lim, start_vel, end_vel);
        std::cout << "planning time: " << (double)(std::clock() - t1) / CLOCKS_PER_SEC << 's' << '\n';

        if (res.ret == 0)
        {
            servo_move_joint(tra, 0, tra.size());
        }
        break;
    }

    case 5: // serve mode 伺服模式 关节空间
            //  meter convert to mm
    {
        joint_pose.jVal[0] = req.pose[0];
        joint_pose.jVal[1] = req.pose[1];
        joint_pose.jVal[2] = req.pose[2];
        joint_pose.jVal[3] = req.pose[3];
        joint_pose.jVal[4] = req.pose[4];
        joint_pose.jVal[5] = req.pose[5];

        speed = (double)req.mvvelo;
        accel = (double)req.mvacc;

        for (int k = 0; k < 6; k++)
        {
            // Fast
            vel_lim[k] = 0.002;  // 0.005
            acc_lim[k] = 0.0005; // 0.0005

            start_vel[k] = 0.002; // 0.002
            end_vel[k] = 0.002;   // 0.002
        }
        robot.get_joint_position(&ref_joint_pose);
        std::clock_t t1 = std::clock();
        Trapezoid_Velocity_Series(ref_joint_pose, joint_pose, tra, vel_lim, acc_lim, start_vel, end_vel);
        std::cout << "planning time: " << (double)(std::clock() - t1) / CLOCKS_PER_SEC << 's' << '\n';
        servo_move_joint(tra, 0, tra.size());
        break;
    }

    default:
        cout << "Please provide move type, 1 for movej, 2 for movep" << endl;
        break;
    }
    std::cout << "multi_move res.ret:  " << res.ret << std::endl;
    cout << "**********************************************" << endl;

    return true;
}

int main(int argc, char **argv)
{
    RobotStatus ret_status;

    //    robot.set_status_data_update_time_interval(40);

    ros::init(argc, argv, "connect_robot_tt");

    ros::NodeHandle n;

    string ip = "192.168.50.170";
    ROS_INFO("Try to connect robot");
    robot.login_in(ip.c_str());
    robot.set_network_exception_handle(110, MOT_ABORT);
    sleep(1);
    robot.power_on();
    sleep(1);
    robot.enable_robot();
    robot.get_robot_status(&ret_status);
    if (ret_status.enabled == false)
    {
        ROS_INFO("restart......");
        robot.power_off();
        sleep(1);
        robot.power_on();
        sleep(1);
        robot.enable_robot();
    }

    ROS_INFO("Robot:%s enable", ip.c_str());
    ros::Duration(1).sleep();
    ROS_INFO("Robot:%s ready!", ip.c_str());

    // 监控状态
    tool_point_pub(n);
    ros::ServiceServer service_multi_move = n.advertiseService("/robot_driver/multi_move", multi_move_callback);
    ros::spin();
    pthread_mutex_destroy(&tool_point_mutex);

    return 0;
}