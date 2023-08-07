#include "ros/ros.h"
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

using namespace std;

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

/* global variables */
JAKAZuRobot robot;

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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_setting_server");
    ros::NodeHandle n;

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

    return 0;
}