#include "ros/ros.h"
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include <cmath>
#include <queue>
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include "robot_msgs/Move.h"
#include "robot_msgs/ServoL.h"
#include "robot_msgs/GetPosition.h"
#include "admittance_control/MDK_msg.h"
#include "admittance_control/Plot.h"
#include "robotiq_ft_sensor/ft_sensor.h"

using namespace std;
using namespace Eigen;

const double PI = 3.1415926;

MatrixXd M(6, 6);
MatrixXd D(6, 6);
MatrixXd K(6, 6);

queue<VectorXd> FTdata_list;
int FTdata_num = 20;

admittance_control::Plot plot_data;

Matrix3d SkewSymmetry(Vector3d vector)
{
    Matrix3d cross_matrix = Matrix3d::Zero();
    cross_matrix(0, 1) = -vector(2);
    cross_matrix(0, 2) = vector(1);
    cross_matrix(1, 0) = vector(2);
    cross_matrix(1, 2) = -vector(0);
    cross_matrix(2, 0) = -vector(1);
    cross_matrix(2, 1) = vector(0);

    return cross_matrix;
}

double AngularPI(double angular)
{
    if (abs(angular) > PI)
        angular = angular - angular / abs(angular) * 2 * PI;

    return angular;
}

static Matrix4d Pose2HomogeneousTransform(VectorXd pose)
{
    Matrix4d T = Matrix4d::Identity();
    Matrix3d rotation_transform;

    rotation_transform = AngleAxisd(pose(5), Vector3d::UnitZ()) *
                         AngleAxisd(pose(4), Vector3d::UnitY()) *
                         AngleAxisd(pose(3), Vector3d::UnitX());
    T.block<3, 3>(0, 0) = rotation_transform;
    T.block<3, 1>(0, 3) = pose.block<3, 1>(0, 0);

    return T;
}

static VectorXd HomogeneousTransform2Pose(Matrix4d T)
{
    VectorXd pose(6);
    Matrix3d rotation_transform;

    pose.block<3, 1>(0, 0) = T.block<3, 1>(0, 3);

    rotation_transform = T.block<3, 3>(0, 0);

    Vector3d n = rotation_transform.col(0);
    Vector3d o = rotation_transform.col(1);
    Vector3d a = rotation_transform.col(2);

    pose(5) = atan2(n(1), n(0));
    pose(4) = atan2(-n(2), n(0) * cos(pose(5)) + n(1) * sin(pose(5)));
    pose(3) = atan2(a(0) * sin(pose(5)) - a(1) * cos(pose(5)), -o(0) * sin(pose(5)) + o(1) * cos(pose(5)));

    pose(3) = AngularPI(pose(3));
    pose(4) = AngularPI(pose(4));
    pose(5) = AngularPI(pose(5));

    return pose;
}

void ForceRecord(const robotiq_ft_sensor::ft_sensor::ConstPtr &msg)
{
    VectorXd FTdata_once(6);

    FTdata_once(0, 0) = msg->Fx;
    FTdata_once(1, 0) = msg->Fy;
    FTdata_once(2, 0) = msg->Fz;
    FTdata_once(3, 0) = msg->Mx;
    FTdata_once(4, 0) = msg->My;
    FTdata_once(5, 0) = msg->Mz;

    FTdata_list.push(FTdata_once);

    if (FTdata_list.size() > FTdata_num)
        FTdata_list.pop();
}

void MDKRecord(const admittance_control::MDK_msg::ConstPtr &msg)
{
    double M_array[36];
    double D_array[36];
    double K_array[36];

    memcpy(M_array, &(msg->M[0]), 8 * 36);
    memcpy(D_array, &(msg->D[0]), 8 * 36);
    memcpy(K_array, &(msg->K[0]), 8 * 36);

    M = Map<Matrix<double, 6, 6, RowMajor>>(M_array);
    D = Map<Matrix<double, 6, 6, RowMajor>>(D_array);
    K = Map<Matrix<double, 6, 6, RowMajor>>(K_array);

    cout << "M修改为:" << M << endl;
    cout << "D修改为:" << D << endl;
    cout << "K修改为:" << K << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "admittance_control");
    ros::NodeHandle n;
    cout.precision(4);

#pragma region /*自定义参数*/
    M = MatrixXd::Identity(6, 6);
    D = MatrixXd::Identity(6, 6);
    K = MatrixXd::Identity(6, 6);

    double M_array[6] = {500, 500, 500, 7.5, 5, 30};
    double D_array[6] = {4000, 5000, 4000, 7.5, 5, 30};
    double K_array[6] = {0, 0, 0, 0, 0, 0};

    for (int i = 0; i < 6; i++)
    {
        M(i, i) = M_array[i];
        K(i, i) = K_array[i];
        D(i, i) = D_array[i];
    }

    VectorXd expected_wrench(6);
    expected_wrench << 0, 0, -40, 0, 0, 0;

    double x_sign = -1;
    double y_sign = 1;
    double x_explore_sign = 1;
    double y_explore_sign = 1; // right:1 left:-1

    double kcontrol_rate = 0.1;
#pragma endregion

#pragma region /*模型参数*/
    int experiement_index = 5;
    VectorXd refer_pose(6);
    VectorXd zero_drift_compensation(6);
    Vector3d G_basis; // 基坐标系重力
    Vector3d centroid_sensor;
    Matrix4d T_pegright2tcp;
    Matrix4d T_sensor2pegright;

    switch (experiement_index)
    {
    case 1:
    case 2:
    {
        /* -0.7 0.03206 0.09872 -3.1415 0 0.812686 */
        if (experiement_index == 1)
            // refer_pose << -0.700, 0.021, 0.248, -3.024, 0.457, 0.750; // rectangle
            refer_pose << -0.720, 0.0, 0.250, -170.0 / 180.0 * PI, 30.0 / 180.0 * PI, 43.0 / 180.0 * PI; // rectangle
        else
            // refer_pose << -0.690, 0.056, 0.280, 2.670, -0.022, 0.919; // rectangle 3nd stage exploration
            refer_pose << -0.730, 0.050, 0.280, 140.0 / 180.0 * PI, -8.0 / 180.0 * PI, 53.0 / 180.0 * PI; // rectangle 3nd stage exploration

        zero_drift_compensation << 29.6454, -29.8912, 153.223, -0.0599064, -1.21829, 1.50143; // rectangle
        G_basis << 0.0, 0.0, -16.8911;                                                        // rectangle
        centroid_sensor << -0.00430543, -0.00220419, 0.00579824;                              // rectangle

        T_pegright2tcp = Matrix4d::Identity();
        T_pegright2tcp(2, 3) = 72.5 / 1000.0;
        T_pegright2tcp(1, 3) = -y_explore_sign * 400.0 / 1000.0;

        T_sensor2pegright = Matrix4d::Identity();
        T_sensor2pegright(2, 3) = -35 / 1000.0;
        T_sensor2pegright(1, 3) = y_explore_sign * 400.0 / 1000.0;
        break;
    }
    case 3:
    case 4:
    {
        if (experiement_index == 3)
            refer_pose << -0.680, -0.109, 0.248, -3.024, 0.457, 0.750; // triangle
        else
            refer_pose << -0.690, -0.074, 0.280, 2.670, -0.022, 0.919; // triangle 3nd stage exploration

        zero_drift_compensation << 29.389, -27.0138, 145.901, -0.0247882, -0.976579, 1.40382; // triangle
        G_basis << 0.0, 0.0, -17.0637;                                                        // triangle
        centroid_sensor << 0.00651398, -0.000348555, 0.00345298;                              // triangle

        T_pegright2tcp = Matrix4d::Identity();
        T_pegright2tcp(2, 3) = 72.5 / 1000.0;
        T_pegright2tcp(1, 3) = -y_explore_sign * 400.0 / 1000.0;

        T_sensor2pegright = Matrix4d::Identity();
        T_sensor2pegright(2, 3) = -35 / 1000.0;
        T_sensor2pegright(1, 3) = y_explore_sign * 400.0 / 1000.0;
        break;
    }
    case 5:
    case 6:
    case 7:
    {
        if (experiement_index == 5)
            refer_pose << -0.720, -0.228, 0.248, -3.048, 0.462, 0.671; // circle-rectangle
        else if (experiement_index == 6)
            refer_pose << -0.740, -0.229, 0.248, -3.024, 0.457, 0.750; // circle-rectangle 2nd stage exploration
        else
            refer_pose << -0.690, -0.194, 0.280, 2.670, -0.022, 0.919; // circle-rectangle 3nd stage exploration

        zero_drift_compensation << 28.5798, -26.5811, 144.498, 0.0608479, -1.10486, 1.41236; // circle-rectangle
        G_basis << 0.0, 0.0, -17.7315;                                                       // circle-rectangle
        centroid_sensor << -0.00420105, -0.00337041, 0.00818461;                             // circle-rectangle

        T_pegright2tcp = Matrix4d::Identity();
        T_pegright2tcp(2, 3) = 72.5 / 1000.0;
        T_pegright2tcp(1, 3) = -y_explore_sign * 400.0 / 1000.0;

        T_sensor2pegright = Matrix4d::Identity();
        T_sensor2pegright(2, 3) = -35 / 1000.0;
        T_sensor2pegright(1, 3) = y_explore_sign * 400.0 / 1000.0;
        break;
    }
    case 8:
    case 9:
    {
        if (experiement_index == 8)
            // refer_pose << -0.610, -0.019, 0.227, 2.7090, -0.1963, 1.0129; // triangle-rectangle exploration
            refer_pose << -0.600, -0.024, 0.227, 2.7822, -0.1719, 1.0280;
        else
            refer_pose << -0.733, -0.057, 0.230, -3.0397, 0.3784, 0.6649; // triangle-rectangle
        // refer_pose << -0.694, -0.035, 0.185, -3.0301, 0.3253, 0.7664; // FT in paper
        zero_drift_compensation << -13.808, -25.0297, 173.615, 0.164186, -0.516528, 0.598439;
        G_basis << 0.0, 0.0, -17.1311;
        centroid_sensor << 0.00135399, 0.00213404, 0.00449994;

        T_pegright2tcp = Matrix4d::Identity();
        T_pegright2tcp(2, 3) = 72.5 / 1000.0;
        T_pegright2tcp(1, 3) = -y_explore_sign * 336.25 / 1000.0;
        T_pegright2tcp(0, 3) = y_explore_sign * 5.05 / 1000.0;

        T_sensor2pegright = Matrix4d::Identity();
        T_sensor2pegright(2, 3) = -35 / 1000.0;
        T_sensor2pegright(1, 3) = y_explore_sign * 336.25 / 1000.0;
        T_sensor2pegright(0, 3) = -y_explore_sign * 5.05 / 1000.0;
        break;
    }
    case 10:
    {
        refer_pose << -0.677, -0.088, 0.256, -3.0263, 0.3766, 0.7298; // circle-rectangle
        zero_drift_compensation << -14.1382, -23.5226, 158.671, 0.30132, -0.529552, 0.532321;
        G_basis << 0.0, 0.0, -17.8754;
        centroid_sensor << 0.000987409, -0.00899561, 0.00602214;

        T_pegright2tcp = Matrix4d::Identity();
        T_pegright2tcp(2, 3) = 72.5 / 1000.0;
        T_pegright2tcp(1, 3) = -y_explore_sign * 278.75 / 1000.0;

        T_sensor2pegright = Matrix4d::Identity();
        T_sensor2pegright(2, 3) = -35 / 1000.0;
        T_sensor2pegright(1, 3) = y_explore_sign * 278.75 / 1000.0;
        break;
    }
    case 11:
    {
        refer_pose << -0.739, -0.124, 0.213, -2.8173, 0.0947, 0.9397; // circle-triangle
        // refer_pose << -0.690, -0.172, 0.259, -3.0994, 0.3573, 0.7107;
        // refer_pose << -0.617, -0.162, 0.251, -2.7378, 0.0289, 0.6962;

        zero_drift_compensation << -12.0182, -23.4407, 160.985, 0.277632, -0.557598, 0.574956;
        G_basis << 0.0, 0.0, -17.6292;
        centroid_sensor << 0.00113796, -0.014999, 0.00587512;

        T_pegright2tcp = Matrix4d::Identity();
        T_pegright2tcp(2, 3) = 72.5 / 1000.0;
        T_pegright2tcp(1, 3) = -y_explore_sign * 342.5 / 1000.0;
        T_pegright2tcp(0, 3) = -y_explore_sign * 5.05 / 1000.0;

        T_sensor2pegright = Matrix4d::Identity();
        T_sensor2pegright(2, 3) = -35 / 1000.0;
        T_sensor2pegright(1, 3) = y_explore_sign * 342.5 / 1000.0;
        T_sensor2pegright(0, 3) = y_explore_sign * 5.05 / 1000.0;
        break;
    }
    }

    Matrix4d T_peg2tcp;
    T_peg2tcp = Matrix4d::Identity();
    T_peg2tcp(2, 3) = 72.5 / 1000.0;

    Matrix4d T_sensor2peg;
    T_sensor2peg = Matrix4d::Identity();
    T_sensor2peg(2, 3) = -35 / 1000.0;

    MatrixXd jacobian_sensor2peg(6, 6);
    jacobian_sensor2peg = MatrixXd::Identity(6, 6);
    jacobian_sensor2peg.block<3, 3>(3, 0) = SkewSymmetry(T_sensor2peg.block<3, 1>(0, 3)) * T_sensor2peg.block<3, 3>(0, 0);
#pragma endregion

#pragma region /*计算参数*/
    Vector3d current_postion = Vector3d::Zero();
    Matrix3d rotation_basis2end = Matrix3d::Identity();
    Matrix4d T_current = Matrix4d::Identity();
    Vector3d G_sensor = Vector3d::Zero();
    VectorXd gravity_compensation(6);
    VectorXd external_wrench_sensor(6);
    queue<VectorXd> FTdata_sum_list;
    VectorXd FTsensor_data(6);
    VectorXd FTdata_sum(6);
    VectorXd delta_wrench(6);
    Vector3d delta_force = Vector3d::Zero();
    Vector3d delta_force_abs = Vector3d::Zero();
    Vector3d delta_torque = Vector3d::Zero();
    Vector3d delta_torque_abs = Vector3d::Zero();
    Matrix4d T_pre = Pose2HomogeneousTransform(refer_pose) * T_peg2tcp;
    Matrix4d T_delta = Matrix4d::Zero();
    VectorXd delta_pose(6);
    delta_pose = VectorXd::Zero(6);
    VectorXd pose_velocity(6);
    VectorXd pose_velocity_sum(6);
    VectorXd pose_acceleration(6);
    Matrix4d T_expected = Matrix4d::Zero();
    VectorXd expected_pose(6);
    VectorXd expected_servo_pose(6);
    VectorXd delta_pose_peg(6);
    Matrix4d T_peg_delta;
    VectorXd start_pose(6);

    int stage = 1; // 一阶段导纳控制单侧入孔、二阶段另一侧入孔判断及搜孔、三阶段导纳控制完成竖直装配，四阶段完成水平装配
    int search_stage = 1;
    bool delta_flag = false;
    bool explore_flag = true;
    long x_count = 0;
    long exceed_count = 0;
    int reach_count = 0;
    double sensor_count = 0.0;

    robot_msgs::ServoL servo_msg;
#pragma endregion

#pragma region /*Reach the Start Pose*/
    ros::Subscriber MDK_sub = n.subscribe<admittance_control::MDK_msg>("/MDK", 1, &MDKRecord);
    ros::Subscriber FTsensor_sub = n.subscribe<robotiq_ft_sensor::ft_sensor>("robotiq_ft_sensor", 20, &ForceRecord);

    ros::Publisher servo_move_pub = n.advertise<robot_msgs::ServoL>("/robot_driver/servo_move", 1);
    ros::Publisher plot_pub = n.advertise<admittance_control::Plot>("/plot_data", 100);

    ros::ServiceClient client_get_position = n.serviceClient<robot_msgs::GetPosition>("/robot_driver/update_position");
    robot_msgs::GetPosition srv_get_position;

    ros::ServiceClient client = n.serviceClient<robot_msgs::Move>("/robot_driver/move_line");
    robot_msgs::Move srv;
    ros::ServiceClient client_stop = n.serviceClient<std_srvs::Empty>("/robot_driver/stop_move");
    std_srvs::Empty srv_stop;

    expected_pose = refer_pose;

    srv.request.pose.clear();
    srv.request.is_block = true;
    srv.request.mvvelo = 0.05;
    srv.request.mvacc = 0.05;
    for (int i = 0; i < 6; i++)
        srv.request.pose.push_back(expected_pose(i));

    if (client.call(srv))
        ROS_INFO("Reach the Start Pose");
    else
    {
        ROS_ERROR("Failed to Reach the Start Pose 1");
        return 1;
    }
#pragma endregion

#pragma region /*FT_Sensor zero compensation*/
    ros::Rate rate(1 / kcontrol_rate);
    sleep(1);

    int num_zero_cali = 0;
    VectorXd zero_cali(6);
    zero_cali = VectorXd::Zero(6);

    while (num_zero_cali < 50)
    {
        /* 更新末端位置 */
        srv_get_position.request.is_tcp_position = true;

        if (client_get_position.call(srv_get_position))
        {
            current_postion << srv_get_position.response.position[0], srv_get_position.response.position[1], srv_get_position.response.position[2];
            rotation_basis2end = AngleAxisd(srv_get_position.response.position[5], Vector3d::UnitZ()) *
                                 AngleAxisd(srv_get_position.response.position[4], Vector3d::UnitY()) *
                                 AngleAxisd(srv_get_position.response.position[3], Vector3d::UnitX()) *
                                 AngleAxisd(PI / 4, Vector3d::UnitZ());
        }
        else
        {
            ROS_ERROR("Failed to Get the Position");
            return 1;
        }

        /* 计算传感器外力 */
        G_sensor = rotation_basis2end.transpose() * G_basis;
        gravity_compensation << G_sensor, centroid_sensor.cross(G_sensor);

        rate.sleep();
        ros::spinOnce();
        sensor_count = 0.0;
        FTdata_sum = VectorXd::Zero(6);
        FTdata_sum_list = FTdata_list;
        while (!FTdata_sum_list.empty())
        {
            FTdata_sum = FTdata_sum + FTdata_sum_list.front();
            FTdata_sum_list.pop();
            sensor_count++;
        }
        FTsensor_data = FTdata_sum / sensor_count;

        external_wrench_sensor = FTsensor_data - zero_drift_compensation;
        external_wrench_sensor = external_wrench_sensor - gravity_compensation;

        zero_cali = zero_cali + external_wrench_sensor;
        num_zero_cali++;
    }

    zero_cali = zero_cali / 50.0;
    cout << zero_cali << endl;

    cout << " OK " << endl;
#pragma endregion

    while (ros::ok())
    {
#pragma region /*Sensor Data Update*/
        /* 更新末端位置 */
        srv_get_position.request.is_tcp_position = true;

        if (client_get_position.call(srv_get_position))
        {
            current_postion << srv_get_position.response.position[0], srv_get_position.response.position[1], srv_get_position.response.position[2];
            rotation_basis2end = AngleAxisd(srv_get_position.response.position[5], Vector3d::UnitZ()) *
                                 AngleAxisd(srv_get_position.response.position[4], Vector3d::UnitY()) *
                                 AngleAxisd(srv_get_position.response.position[3], Vector3d::UnitX()) *
                                 AngleAxisd(PI / 4, Vector3d::UnitZ());
            T_current = Matrix4d::Identity();
            T_current.block<3, 3>(0, 0) = rotation_basis2end;
            T_current.block<3, 1>(0, 3) = current_postion;
            T_current = T_current * T_peg2tcp;
        }
        else
        {
            ROS_ERROR("Failed to Get the Position");
            return 1;
        }

        /* 计算传感器外力 */
        G_sensor = rotation_basis2end.transpose() * G_basis;
        gravity_compensation << G_sensor, centroid_sensor.cross(G_sensor);

        rate.sleep();
        ros::spinOnce();
        sensor_count = 0.0;
        FTdata_sum = VectorXd::Zero(6);
        FTdata_sum_list = FTdata_list;
        while (!FTdata_sum_list.empty())
        {
            FTdata_sum = FTdata_sum + FTdata_sum_list.front();
            FTdata_sum_list.pop();
            sensor_count++;
        }
        FTsensor_data = FTdata_sum / sensor_count;

        external_wrench_sensor = FTsensor_data - zero_drift_compensation;
        external_wrench_sensor = external_wrench_sensor - gravity_compensation - zero_cali;

        /* 计算工件坐标系下deltaF */
        delta_wrench = external_wrench_sensor - jacobian_sensor2peg.inverse() * expected_wrench;

        delta_force = delta_wrench.block<3, 1>(0, 0);
        delta_force_abs = delta_force.array().abs();
        delta_torque = delta_wrench.block<3, 1>(3, 0);
        delta_torque_abs = delta_torque.array().abs();

        delta_wrench = jacobian_sensor2peg * delta_wrench;
#pragma endregion

        switch (stage)
        {
        case 1:
        {
            delta_wrench(0) = 0;
            delta_wrench(1) = 0;
            delta_wrench(3) = 0;
            delta_wrench(4) = 0;
            delta_wrench(5) = 0;
            break;
        }
        case 2:
        {
            delta_wrench(3) = 0;
            delta_wrench(4) = 0;
            delta_wrench(5) = 0;

            T_peg_delta = Pose2HomogeneousTransform(start_pose).inverse() * Pose2HomogeneousTransform(expected_pose);
            delta_pose_peg = HomogeneousTransform2Pose(T_peg_delta);

            if (abs(delta_pose_peg(1)) >= 0.1 || delta_flag)
            {
                explore_flag = false;
                delta_wrench(1) = 0;

                if (!delta_flag) // first time
                {
                    start_pose = expected_pose;
                    expected_wrench << y_explore_sign * 40, 0, -5, 0, 0, 0;
                }

                if (delta_flag && abs(delta_pose_peg(0)) >= 0.02) // second time
                {
                    if (y_sign > 0)
                    {
                        expected_wrench << 0, -40, -5, 0, 0, 0;
                        y_sign = -1;
                    }
                    else
                    {
                        expected_wrench << 0, 40, -5, 0, 0, 0;
                        y_sign = 1;
                    }
                    delta_flag = false;
                    start_pose = expected_pose;
                    break;
                }

                delta_flag = true;
            }
            else
            {
                explore_flag = true;
                delta_wrench(0) = 0;

                if (delta_force(2) > 4)
                    exceed_count++;
                else
                    exceed_count = 0;

                if (exceed_count > 10)
                    delta_wrench(1) = 0;
            }
            break;
        }
        case 3:
        {
            delta_wrench(3) = 0;
            delta_wrench(4) = 0;
            delta_wrench(5) = 0;

            T_peg_delta = Pose2HomogeneousTransform(start_pose).inverse() * Pose2HomogeneousTransform(expected_pose);
            delta_pose_peg = HomogeneousTransform2Pose(T_peg_delta);

            // if (x_sign < 0)
            // {
            //     if (delta_force(1) < -15)
            //         x_count++;
            //     else
            //         x_count = 0;

            //     if (abs(delta_pose_peg(0)) >= 0.1 || x_count >= 3)
            //     {
            //         x_sign = 1;
            //         expected_wrench << 40, 5, -5, 0, 0, 0;
            //     }
            // }
            break;
        }
        case 4:
        {
            delta_wrench(3) = 0;
            delta_wrench(4) = 0;
            delta_wrench(5) = 0;
            break;
        }
        case 5:
        {
            delta_wrench(4) = 0;
            delta_wrench(5) = 0;
            break;
        }
        case 6:
        {
            delta_wrench(4) = 0;

            if (delta_force_abs(0) > 5 || delta_force_abs(2) > 8 || delta_torque_abs(0) > 1.5)
                exceed_count++;
            else
                exceed_count = 0;

            if (exceed_count > 5)
                delta_wrench(5) = 0;
            break;
        }
        default:
            break;
        }

#pragma region /*Admittance Computation/ \
        /* 计算xt,dotxt,dotdotxt */
        T_delta = T_current.inverse() * T_pre;
        pose_velocity = -HomogeneousTransform2Pose(T_delta) / kcontrol_rate;
        pose_acceleration = M.inverse() * (delta_wrench - D * pose_velocity);

        T_pre = T_current;

        /* 计算xt+1,期望位姿 */
        delta_pose = pose_velocity * kcontrol_rate + pose_acceleration * kcontrol_rate * kcontrol_rate;
        T_delta = Pose2HomogeneousTransform(delta_pose);
        T_expected = T_current * T_delta * T_peg2tcp.inverse();
        expected_pose = HomogeneousTransform2Pose(T_expected);

        /* 数据观测 */
        for (int i = 0; i < 6; i++)
        {
            plot_data.data[i + 6] = srv_get_position.response.position[i];
            plot_data.data[i + 12] = expected_servo_pose[i];
            plot_data.data[i + 18] = external_wrench_sensor[i];
            plot_data.data[i + 24] = delta_wrench[i];
            plot_data.data[i + 30] = pose_velocity[i];
            plot_data.data[i + 36] = pose_acceleration(i);
            plot_data.data[i + 42] = delta_pose[i] / kcontrol_rate;
            plot_data.data[i + 48] = start_pose[i];
            plot_data.data[i + 54] = (jacobian_sensor2peg * external_wrench_sensor)[i];
        }

        plot_data.data[0] = stage;
        plot_data.data[1] = search_stage;
        plot_data.data[2] = external_wrench_sensor[3] + external_wrench_sensor[1] * 0.035 + external_wrench_sensor[2] * 0.4;
        plot_data.data[3] = external_wrench_sensor[4] - external_wrench_sensor[0] * 0.035;
        plot_data.data[4] = external_wrench_sensor[5] - external_wrench_sensor[0] * 0.4;
#pragma endregion

        /* 运动切换 */
        switch (stage)
        {
        case 1: // 下压
        {
            if (delta_force_abs(2) < 35)
                expected_wrench << 0, 0, -5, 0, 0, 0;

            if (delta_force_abs(2) < 1)
            {
                if (reach_count <= 10)
                    reach_count++;
                else
                {
                    stage = 2;
                    expected_wrench << 0, y_sign * 40, -5, 0, 0, 0;
                    start_pose = expected_pose;
                    reach_count = 0;

                    double M_array[6] = {500, 500, 100, 7.5, 5, 30};
                    double D_array[6] = {4000, 5000, 1000, 7.5, 5, 30};
                    double K_array[6] = {0, 0, 0, 0, 0, 0};

                    for (int i = 0; i < 6; i++)
                    {
                        M(i, i) = M_array[i];
                        K(i, i) = K_array[i];
                        D(i, i) = D_array[i];
                    }
                }
            }
            break;
        }
        case 2: // 左右探索
        {
            // if (delta_force_abs(1) < 30 && explore_flag && y_sign < 0)
            // {
            //     expected_wrench << 0, 40, -5, 0, 0, 0;
            //     y_sign = 1;
            // }
            if (delta_force_abs(1) < 25 && explore_flag) // 20为防止运动摩擦力影响
                expected_wrench << 0, y_sign * 5, -5, 0, 0, 0;
            if (delta_force_abs(1) < 2 && delta_force_abs(2) < 1 && explore_flag)
            {
                if (reach_count <= 10)
                    reach_count++;
                else
                {
                    stage = 3;
                    expected_wrench << x_sign * 35, y_sign * 5, -5, 0, 0, 0; // 20为下一步检测离孔
                    start_pose = expected_pose;
                    reach_count = 0;

                    double M_array[6] = {500, 100, 100, 7.5, 5, 30};
                    double D_array[6] = {4000, 1000, 1000, 7.5, 5, 30};
                    double K_array[6] = {0, 0, 0, 0, 0, 0};

                    for (int i = 0; i < 6; i++)
                    {
                        M(i, i) = M_array[i];
                        K(i, i) = K_array[i];
                        D(i, i) = D_array[i];
                    }
                }
            }
            break;
        }
        case 3: // 前后探索
        {
            if (delta_force_abs(0) < 15) // 10为防止运动摩擦力影响
                expected_wrench << x_sign * 5, y_sign * 5, -5, 0, 0, 0;
            if (delta_force_abs(0) < 1 && delta_force_abs(1) < 1 && delta_force_abs(2) < 1)
            {
                if (reach_count <= 10)
                    reach_count++;
                else
                {
                    stage = 4;
                    expected_wrench << x_sign * 5, y_sign * 5, -5, 0, 0, 0;
                    start_pose = expected_pose;
                    reach_count = 0;

                    T_peg2tcp = T_pegright2tcp;

                    T_pre = Matrix4d::Identity();
                    T_pre.block<3, 3>(0, 0) = rotation_basis2end;
                    T_pre.block<3, 1>(0, 3) = current_postion;
                    T_pre = T_pre * T_peg2tcp;

                    T_sensor2peg = T_sensor2pegright;

                    jacobian_sensor2peg = MatrixXd::Identity(6, 6);
                    jacobian_sensor2peg.block<3, 3>(3, 0) = SkewSymmetry(T_sensor2peg.block<3, 1>(0, 3)) * T_sensor2peg.block<3, 3>(0, 0);

                    double M_array[6] = {500, 500, 2000, 50, 5, 200};
                    double D_array[6] = {250, 250, 1000, 50, 5, 200};
                    double K_array[6] = {0, 0, 0, 0, 0, 0};

                    for (int i = 0; i < 6; i++)
                    {
                        M(i, i) = M_array[i];
                        K(i, i) = K_array[i];
                        D(i, i) = D_array[i];
                    }
                }
            }
            break;
        }
        case 4: // 切换柔顺中心，调整接触力
        {
            if (delta_force_abs.maxCoeff() < 5)
            {
                if (reach_count <= 10)
                    reach_count++;
                else
                {
                    stage = 5;
                    expected_wrench << x_sign * 5, y_sign * 5, -10, -y_sign * 3, 0, 0;
                    start_pose = expected_pose;
                    reach_count = 0;
                }
            }
            else
                reach_count = 0;
            break;
        }
        case 5: // 左右旋转
        {
            if (delta_force_abs.maxCoeff() < 5 && abs(delta_torque(0)) < 0.5)
            {
                if (reach_count <= 10)
                    reach_count++;
                else
                {
                    ROS_INFO("Finish the first stage");

                    stage = 6;
                    search_stage = 1;
                    expected_wrench << x_sign * 10, y_sign * 5, -10, -y_sign * 3, 0, 8; // + y_sign * x_sign * 10 * 0.25;

                    reach_count = 0;
                }
            }
            else
                reach_count = 0;
            break;
        }
        case 6: // 前后旋转
        {
            switch (search_stage)
            {
            case 1:
                if (delta_torque_abs(2) < 3)
                {
                    expected_wrench << x_sign * 10, y_sign * 5, -10, -y_sign * 3, 0, 3;
                }
                if (delta_force_abs.maxCoeff() < 5 && delta_torque_abs(2) < 1.0)
                {
                    if (reach_count <= 10)
                        reach_count++;
                    else
                    {
                        ROS_INFO("Finish the second stage");

                        stage = 7;
                        expected_wrench << x_sign * 10, 0, -40, 0, 0, 0;

                        T_peg2tcp = Matrix4d::Identity();
                        T_peg2tcp(2, 3) = 72.5 / 1000.0;

                        T_pre = Matrix4d::Identity();
                        T_pre.block<3, 3>(0, 0) = rotation_basis2end;
                        T_pre.block<3, 1>(0, 3) = current_postion;
                        T_pre = T_pre * T_peg2tcp;

                        T_sensor2peg = Matrix4d::Identity();
                        T_sensor2peg(2, 3) = -35 / 1000.0;

                        jacobian_sensor2peg = MatrixXd::Identity(6, 6);
                        jacobian_sensor2peg.block<3, 3>(3, 0) = SkewSymmetry(T_sensor2peg.block<3, 1>(0, 3)) * T_sensor2peg.block<3, 3>(0, 0);

                        double M_array[6] = {500, 500, 500, 7.5, 5, 30};
                        double D_array[6] = {250, 250, 250, 7.5, 5, 30};
                        double K_array[6] = {0, 0, 0, 0, 0, 0};

                        for (int i = 0; i < 6; i++)
                        {
                            M(i, i) = M_array[i];
                            K(i, i) = K_array[i];
                            D(i, i) = D_array[i];
                        }
                    }
                }
                else if (expected_pose(5) - start_pose(5) > 30.0 / 180.0 * PI || delta_torque(2) > 6.0)
                {
                    search_stage = 2;
                    expected_wrench << x_sign * 10, y_sign * 10, -10, -y_sign * 3, 0, -8;
                }
                break;
            case 2:
                if (delta_torque_abs(2) < 5.0)
                {
                    expected_wrench << x_sign * 10, y_sign * 10, -10, -y_sign * 3, 0, -3;
                }
                if (delta_force_abs.maxCoeff() < 5 && delta_torque_abs(2) < 1.0)
                {
                    if (reach_count <= 10)
                        reach_count++;
                    else
                    {
                        ROS_INFO("Finish the second stage");

                        stage = 7;
                        expected_wrench << 0, 0, -40, 0, 0, 0;

                        T_peg2tcp = Matrix4d::Identity();
                        T_peg2tcp(2, 3) = 72.5 / 1000.0;

                        T_pre = Matrix4d::Identity();
                        T_pre.block<3, 3>(0, 0) = rotation_basis2end;
                        T_pre.block<3, 1>(0, 3) = current_postion;
                        T_pre = T_pre * T_peg2tcp;

                        T_sensor2peg = Matrix4d::Identity();
                        T_sensor2peg(2, 3) = -35 / 1000.0;

                        jacobian_sensor2peg = MatrixXd::Identity(6, 6);
                        jacobian_sensor2peg.block<3, 3>(3, 0) = SkewSymmetry(T_sensor2peg.block<3, 1>(0, 3)) * T_sensor2peg.block<3, 3>(0, 0);

                        double M_array[6] = {500, 500, 500, 7.5, 5, 30};
                        double D_array[6] = {250, 250, 250, 7.5, 5, 30};
                        double K_array[6] = {0, 0, 0, 0, 0, 0};

                        for (int i = 0; i < 6; i++)
                        {
                            M(i, i) = M_array[i];
                            K(i, i) = K_array[i];
                            D(i, i) = D_array[i];
                        }
                    }
                }
                else if (start_pose(5) - expected_pose(5) > 30.0 / 180.0 * PI)
                {
                    ROS_ERROR("Failed to Find the Hole");
                    return 1;
                }
                break;
            default:
                break;
            }
            break;
        }
        case 7: // 上下旋转，切换柔顺中心
        {
            if (delta_force_abs.maxCoeff() < 5 && delta_torque_abs.maxCoeff() < 0.2)
            {
                if (reach_count <= 10)
                    reach_count++;
                else
                {
                    ROS_INFO("Completed");
                    servo_msg.servo_mode = false;
                    servo_move_pub.publish(servo_msg);

                    return 0;
                }
            }
            else
                reach_count = 0;
            break;
        }
        }

        // 将虚拟TCP位置旋转45度发给机械臂
        T_expected = Matrix4d::Identity();
        Matrix3d temp_rotation;
        temp_rotation = AngleAxisd(-PI / 4, Vector3d::UnitZ());
        T_expected.block<3, 3>(0, 0) = temp_rotation;
        T_expected = Pose2HomogeneousTransform(expected_pose) * T_expected;
        expected_servo_pose = HomogeneousTransform2Pose(T_expected);

        plot_pub.publish(plot_data);

        servo_msg.servo_mode = true;
        for (int i = 0; i < 6; i++)
            servo_msg.pose[i] = expected_servo_pose(i);
        servo_move_pub.publish(servo_msg);
    }

    servo_msg.servo_mode = false;
    servo_move_pub.publish(servo_msg);

    return 0;
}
