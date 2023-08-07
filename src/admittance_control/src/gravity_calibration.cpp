#include <sstream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/WrenchStamped.h"
#include "robot_msgs/Move.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "time.h"

using namespace std;

const double PI = 3.1415926;

Eigen::Matrix3d Transform_Basis2End;
Eigen::MatrixXd F_A(10 * 3, 4);
Eigen::MatrixXd M_A(10 * 3, 6);
Eigen::MatrixXd FT_Record(6, 1);
Eigen::MatrixXd FT_Sum(6, 1);
Eigen::MatrixXd F_measure(10 * 3, 1);
Eigen::MatrixXd M_measure(10 * 3, 1);
Eigen::MatrixXd F_parameter(4, 1);
Eigen::MatrixXd M_parameter(6, 1);
Eigen::MatrixXd Test_Joint_Angle(10, 6);
robot_msgs::Move srv;

void ForceRecord(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
    FT_Record(0, 0) = msg->wrench.force.x;
    FT_Record(1, 0) = msg->wrench.force.y;
    FT_Record(2, 0) = msg->wrench.force.z;
    FT_Record(3, 0) = msg->wrench.torque.x;
    FT_Record(4, 0) = msg->wrench.torque.y;
    FT_Record(5, 0) = msg->wrench.torque.z;
    ROS_INFO("I heard: force[%f, %f, %f] torque[%f, %f, %f]",
             msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
             msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z);
}
void ToolPointRecord(const geometry_msgs::TwistStamped &msg)
{
    Transform_Basis2End = Eigen::AngleAxisd(msg.twist.angular.z, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(msg.twist.angular.y, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(msg.twist.angular.x, Eigen::Vector3d::UnitX());
    ROS_INFO("I heard: linear[%f, %f, %f] angular[%f, %f, %f]",
             msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,
             msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z);
    // cout << Transform_Basis2End << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "admittance_control");

    ros::NodeHandle n;

    ros::ServiceClient client = n.serviceClient<robot_msgs::Move>("/robot_driver/move_joint");

    ros::Subscriber FTsensor_sub = n.subscribe("netft_data", 1, ForceRecord);
    ros::Subscriber servo_line_sub = n.subscribe("/robot_driver/tool_point", 1, ToolPointRecord);

    F_A = Eigen::MatrixXd::Zero(10 * 3, 4);
    M_A = Eigen::MatrixXd::Zero(10 * 3, 6);
    Test_Joint_Angle << 0.1652374610684933, 1.854585041861113, 1.898176046369952, 0.9835086341685395, -1.5764230571325082, 1.7271518570372968,
        0.16542920866312294, 1.8091168934845507, 1.706895836502185, 1.914838685509543, -1.5775016373523, 1.7271518570372968,
        0.1629604583822659, 1.810063647233035, 1.8530314720593273, 0.9035019503093071, -1.6826032876586914, 2.1433639796303385,
        0.24077402912792148, 1.7577764750224547, 1.8510061380910512, 1.09290063690477, -1.8478777300048084, 1.883006698797231,
        0.24075006067859278, 1.7577644907977907, 1.8510061380910512, 1.0930084949267491, -1.4051924551282067, 1.883006698797231,
        0.2247151680776861, 1.7581360017623855, 1.8515454282009471, 1.2406541427916002, -1.5903367419678238, 1.883006698797231,
        0.22596152744277898, 1.7586633076476172, 1.8516892388969193, 0.8232196292827942, -1.6298846833601948, 1.8828389196519297,
        0.23514144353567473, 1.758519496951645, 1.8520847183108429, 1.1363913882117134, -1.5420642850198027, 1.8154755928135917,
        0.23729860397525857, 1.7584595758283228, 1.8520008287381926, 1.2343264721688207, -1.6071146564979206, 1.7954739218487836,
        0.23517739620966782, 1.8432000284299759, 1.8521925763328224, 1.1010618939011956, -1.5350295451418268, 1.7621937299558703;

    for (int i = 0; i < 10; i++)
    {
        FT_Sum = Eigen::MatrixXd::Zero(6, 1);

        srv.request.pose.clear();
        srv.request.is_block = true;
        srv.request.mvvelo = 0.5;
        srv.request.mvacc = 0.05;
        srv.request.pose.push_back(Test_Joint_Angle(i, 0));
        srv.request.pose.push_back(Test_Joint_Angle(i, 1));
        srv.request.pose.push_back(Test_Joint_Angle(i, 2));
        srv.request.pose.push_back(Test_Joint_Angle(i, 3));
        srv.request.pose.push_back(Test_Joint_Angle(i, 4));
        srv.request.pose.push_back(Test_Joint_Angle(i, 5));

        if (client.call(srv))
        {
            ROS_INFO("Response from server message: %s",
                     srv.response.message.c_str());
            sleep(3);
            for (int j = 0; j < 10; j++)
            {
                ros::spinOnce();
                FT_Sum = FT_Sum + FT_Record;
                // cout << FT_Sum << endl;
                sleep(1);
            }
            F_measure(3 * i, 0) = FT_Sum(0, 0) / 10.0;
            F_measure(3 * i + 1, 0) = FT_Sum(1, 0) / 10.0;
            F_measure(3 * i + 2, 0) = FT_Sum(2, 0) / 10.0;
            M_measure(3 * i, 0) = FT_Sum(3, 0) / 10.0;
            M_measure(3 * i + 1, 0) = FT_Sum(4, 0) / 10.0;
            M_measure(3 * i + 2, 0) = FT_Sum(5, 0) / 10.0;
            F_A(3 * i, 0) = Transform_Basis2End(2, 0);
            F_A(3 * i + 1, 0) = Transform_Basis2End(2, 1);
            F_A(3 * i + 2, 0) = Transform_Basis2End(2, 2);
            F_A(3 * i, 1) = 1;
            F_A(3 * i + 1, 2) = 1;
            F_A(3 * i + 2, 3) = 1;
        }
        else
        {
            ROS_ERROR("failed to call /robot_driver/move_line");
            return 1;
        }
    }

    // cout << F_A << endl;
    // cout << F_measure << endl;

    F_parameter = F_A.transpose() * F_A;
    F_parameter = F_parameter.inverse() * F_A.transpose() * F_measure;

    cout << "Measured gravity is : " << F_parameter(0, 0) << endl;
    cout << "Zero drift of force is : ( " << F_parameter(1, 0) << ", "
         << F_parameter(2, 0) << ", " << F_parameter(3, 0) << ")" << endl;

    for (int i = 0; i < 10; i++)
    {
        M_A(3 * i, 1) = F_parameter(0, 0) * F_A(3 * i + 2, 0);
        M_A(3 * i, 2) = -F_parameter(0, 0) * F_A(3 * i + 1, 0);
        M_A(3 * i + 1, 0) = -F_parameter(0, 0) * F_A(3 * i + 2, 0);
        M_A(3 * i + 1, 2) = F_parameter(0, 0) * F_A(3 * i, 0);
        M_A(3 * i + 2, 0) = F_parameter(0, 0) * F_A(3 * i + 1, 0);
        M_A(3 * i + 2, 1) = -F_parameter(0, 0) * F_A(3 * i, 0);
        M_A(3 * i, 3) = 1;
        M_A(3 * i + 1, 4) = 1;
        M_A(3 * i + 2, 5) = 1;
    }

    // cout << M_A << endl;
    // cout << M_measure << endl;

    M_parameter = M_A.transpose() * M_A;
    M_parameter = M_parameter.inverse() * M_A.transpose() * M_measure;

    cout << "Coordinate of centroid is : ( " << M_parameter(0, 0) << ", "
         << M_parameter(1, 0) << ", " << M_parameter(2, 0) << ")" << endl;
    cout << "Zero drift of torque is : ( " << M_parameter(3, 0) << ", "
         << M_parameter(4, 0) << ", " << M_parameter(5, 0) << ")" << endl;

    return 0;
}