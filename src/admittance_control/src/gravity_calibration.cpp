#include <sstream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/WrenchStamped.h"
#include "robot_msgs/Move.h"
#include "robot_msgs/GetPosition.h"
#include "robotiq_ft_sensor/ft_sensor.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "time.h"

using namespace std;
using namespace Eigen;

const double PI = 3.1415926;

Eigen::Matrix3d Transform_End2Basis;
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

Matrix3d SkewSymmetry(MatrixXd vector)
{
    Matrix3d cross_matrix = Matrix3d::Zero();
    cross_matrix(0, 1) = -vector(2, 0);
    cross_matrix(0, 2) = vector(1, 0);
    cross_matrix(1, 0) = vector(2, 0);
    cross_matrix(1, 2) = -vector(0, 0);
    cross_matrix(2, 0) = -vector(1, 0);
    cross_matrix(2, 1) = vector(0, 0);

    return cross_matrix;
}

void ForceRecord(const robotiq_ft_sensor::ft_sensor::ConstPtr &msg)
{
    FT_Record(0, 0) = msg->Fx;
    FT_Record(1, 0) = msg->Fy;
    FT_Record(2, 0) = msg->Fz;
    FT_Record(3, 0) = msg->Mx;
    FT_Record(4, 0) = msg->My;
    FT_Record(5, 0) = msg->Mz;
    ROS_INFO("I heard: force[%f, %f, %f] torque[%f, %f, %f]",
             msg->Fx, msg->Fy, msg->Fz,
             msg->Mx, msg->My, msg->Mz);
}
void ToolPointRecord(const geometry_msgs::TwistStamped &msg)
{
    Transform_End2Basis = Eigen::AngleAxisd(msg.twist.angular.z, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(msg.twist.angular.y, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(msg.twist.angular.x, Eigen::Vector3d::UnitX());
    ROS_INFO("I heard: linear[%f, %f, %f] angular[%f, %f, %f]",
             msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,
             msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z);
    // cout << Transform_End2Basis << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "admittance_control");

    ros::NodeHandle n;

    ros::ServiceClient client = n.serviceClient<robot_msgs::Move>("/robot_driver/move_joint");

    ros::Subscriber FTsensor_sub = n.subscribe("robotiq_ft_sensor", 1, ForceRecord);
    ros::Subscriber servo_line_sub = n.subscribe("/robot_driver/tool_point", 1, ToolPointRecord);
    ros::ServiceClient client_get_position = n.serviceClient<robot_msgs::GetPosition>("/robot_driver/update_position");
    robot_msgs::GetPosition srv_get_position;

    F_A = Eigen::MatrixXd::Zero(10 * 3, 4);
    M_A = Eigen::MatrixXd::Zero(10 * 3, 6);

    Test_Joint_Angle << 0.16564492470708136, 1.9876339040847797, 1.9052227704725921, 0.7841390726523334, -1.5768664734450892, 1.7272716992839405 - PI / 4,
        0.16556103513443088, 1.9220082898227726, 2.0517898381176516, 0.38597519240381006, -1.5769982999163972, 1.7274394784292415 - PI / 4,
        0.16556103513443088, 1.9982639113620624, 1.6930660412395193, 1.4163189079217124, -1.5158188330048659, 1.7274155099799127 - PI / 4,
        0.1429947400914508, 1.964024981496001, 1.8941733153320572, 0.7726941380978746, -1.504002387485812, 1.7274155099799127 - PI / 4,
        0.1946587326194843, 1.917418331776325, 1.8758134831462658, 0.9394066874037139, -1.694036237988486, 1.7274394784292415 - PI / 4,
        0.13187337960292958, 1.923769970848433, 1.905450470741215, 0.8614612901867504, -1.5042660404284276, 1.6411051239472296 - PI / 4,
        0.13206512719755925, 1.9237819550730975, 1.9053306284945715, 0.8615571639840652, -1.624455829587242, 1.8829587618985737 - PI / 4,
        0.1579510524725656, 1.9115939985894488, 2.0419627738928807, 0.39955331894852397, -1.5201211696593693, 1.7533373879289122 - PI / 4,
        0.16381133833343509, 1.9268019796885147, 1.7770155350133248, 1.3250470528779865, -1.5643789113448314, 1.8826591562819646 - PI / 4,
        0.15473928026251849, 1.9989470121679305, 1.9246252302041824, 0.7967944138978921, -1.5699515758137563, 1.723340873594032 - PI / 4;

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
            for (int j = 0; j < 20; j++)
            {
                ros::spinOnce();
                FT_Sum = FT_Sum + FT_Record;
                // cout << FT_Sum << endl;
                sleep(1);
            }

            srv_get_position.request.is_tcp_position = true;

            if (client_get_position.call(srv_get_position))
            {
                Transform_End2Basis = AngleAxisd(srv_get_position.response.position[5], Vector3d::UnitZ()) *
                                      AngleAxisd(srv_get_position.response.position[4], Vector3d::UnitY()) *
                                      AngleAxisd(srv_get_position.response.position[3], Vector3d::UnitX()) *
                                      AngleAxisd(PI / 4, Vector3d::UnitZ());
            }
            else
            {
                ROS_ERROR("Failed to Get the Position");
                return 1;
            }
            F_measure(3 * i, 0) = FT_Sum(0, 0) / 20.0;
            F_measure(3 * i + 1, 0) = FT_Sum(1, 0) / 20.0;
            F_measure(3 * i + 2, 0) = FT_Sum(2, 0) / 20.0;
            M_measure(3 * i, 0) = FT_Sum(3, 0) / 20.0;
            M_measure(3 * i + 1, 0) = FT_Sum(4, 0) / 20.0;
            M_measure(3 * i + 2, 0) = FT_Sum(5, 0) / 20.0;
            F_A(3 * i, 0) = Transform_End2Basis(2, 0);
            F_A(3 * i + 1, 0) = Transform_End2Basis(2, 1);
            F_A(3 * i + 2, 0) = Transform_End2Basis(2, 2);
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