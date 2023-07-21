#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include <cmath>
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include "robot_msgs/Move.h"
#include "admittance_control/reconfigureConfig.h"
#include "dynamic_reconfigure/server.h"
#include "dynamic_model.h"
#include "MDK_computation.h"

using namespace std;
using namespace Eigen;

const double PI = 3.1415926;

#pragma region /* FTsensor calibration */
Matrix<double, 6, 1> zero_drift_compensation;
Matrix<double, 6, 1> FTsensor_origin_data;
Matrix<double, 6, 1> FTsensor_actual_data;
#pragma endregion

/* Gravity compensation */
Matrix3d transform_basis2end;
Matrix<double, 3, 1> G_basis;
Matrix<double, 3, 1> G_sensor;
Matrix<double, 3, 1> M_sensor;
Matrix<double, 3, 1> centroid_sensor;
Matrix<double, 6, 1> gravity_compensation;
Matrix<double, 6, 1> external_wrench_sensor;
Matrix<double, 3, 1> external_force_sensor;
Matrix<double, 3, 1> external_torque_sensor;

/* Admittance transform*/
Matrix<double, 3, 1> temp1;
Matrix<double, 3, 1> temp2;
Matrix<double, 6, 1> temp3;
Matrix<double, 6, 4> MDH;
Matrix<double, 6, 6> M_referance;
Matrix<double, 60, 1> dynamic_parameter;
Matrix<double, 6, 6> transform_sensor2end;
Matrix<double, 6, 1> external_wrench_end;
Matrix<double, 6, 1> current_pose;
Matrix<double, 4, 4> homogeneous_transform_current;
Matrix<double, 6, 1> expected_wrench;
Matrix<double, 6, 1> origin_expected_pose;
Matrix<double, 4, 4> homogeneous_transform_expected;
Matrix<double, 6, 1> origin_expected_joint;
Matrix<double, 6, 1> delta_wrench;
Matrix<double, 4, 4> delta_homogeneous_transform;
Matrix3d delta_rotation;
Matrix<double, 3, 1> delta_angular;
Matrix<double, 6, 1> delta_pose;
Matrix<double, 6, 1> delta_pose_velocity;
Matrix<double, 6, 1> delta_pose_acceleration;
Matrix<double, 6, 1> pre_delta_pose;
MatrixXd M(6, 6);
MatrixXd D(6, 6);
MatrixXd K(6, 6);
Matrix<double, 6, 1> expected_pose;
double kcontrol_rate = 0.08;
robot_msgs::Move srv;

Matrix3d SkewSymmetry(Matrix<double, 3, 1> vector)
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

double AngularPI(double angular)
{
    if (abs(angular) > PI)
        angular = angular - angular / abs(angular) * 2 * PI;

    return angular;
}

static MatrixXd R2rpy(const Matrix3d R)
{
    Vector3d n = R.col(0);
    Vector3d o = R.col(1);
    Vector3d a = R.col(2);

    Matrix<double, 3, 1> rpy;
    double y = atan2(n(1), n(0));
    double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
    double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
    rpy(0, 0) = r;
    rpy(1, 0) = p;
    rpy(2, 0) = y;

    return rpy;
}

void ToolPointRecord(const geometry_msgs::TwistStamped &msg)
{
    current_pose(0, 0) = msg.twist.linear.x;
    current_pose(1, 0) = msg.twist.linear.y;
    current_pose(2, 0) = msg.twist.linear.z;
    current_pose(3, 0) = msg.twist.angular.x;
    current_pose(4, 0) = msg.twist.angular.y;
    current_pose(5, 0) = msg.twist.angular.z;
    transform_basis2end = AngleAxisd(msg.twist.angular.z, Vector3d::UnitZ()) *
                          AngleAxisd(msg.twist.angular.y, Vector3d::UnitY()) *
                          AngleAxisd(msg.twist.angular.x, Vector3d::UnitX());
    homogeneous_transform_current << transform_basis2end(0, 0), transform_basis2end(0, 1), transform_basis2end(0, 2), current_pose(0, 0),
        transform_basis2end(1, 0), transform_basis2end(1, 1), transform_basis2end(1, 2), current_pose(1, 0),
        transform_basis2end(2, 0), transform_basis2end(2, 1), transform_basis2end(2, 2), current_pose(2, 0),
        0, 0, 0, 1;
}

void Force2Pose(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
    FTsensor_origin_data(0, 0) = msg->wrench.force.x;
    FTsensor_origin_data(1, 0) = msg->wrench.force.y;
    FTsensor_origin_data(2, 0) = msg->wrench.force.z;
    FTsensor_origin_data(3, 0) = msg->wrench.torque.x;
    FTsensor_origin_data(4, 0) = msg->wrench.torque.y;
    FTsensor_origin_data(5, 0) = msg->wrench.torque.z;

    FTsensor_actual_data = FTsensor_origin_data - zero_drift_compensation;
}

void CallbackFunc(admittance_control::reconfigureConfig &ConfigType_obj, uint32_t level)
{
    M(0, 0) = ConfigType_obj.M_F_x;
    M(1, 1) = ConfigType_obj.M_F_y;
    M(2, 2) = ConfigType_obj.M_F_z;
    M(3, 3) = ConfigType_obj.M_T_x;
    M(4, 4) = ConfigType_obj.M_T_y;
    M(5, 5) = ConfigType_obj.M_T_z;

    // D(0, 0) = ConfigType_obj.D_F_x;
    // D(1, 1) = ConfigType_obj.D_F_y;
    // D(2, 2) = ConfigType_obj.D_F_z;
    // D(3, 3) = ConfigType_obj.D_T_x;
    // D(4, 4) = ConfigType_obj.D_T_y;
    // D(5, 5) = ConfigType_obj.D_T_z;

    K(0, 0) = ConfigType_obj.K_F_x;
    K(1, 1) = ConfigType_obj.K_F_y;
    K(2, 2) = ConfigType_obj.K_F_z;
    K(3, 3) = ConfigType_obj.K_T_x;
    K(4, 4) = ConfigType_obj.K_T_y;
    K(5, 5) = ConfigType_obj.K_T_z;

    MDK MDK_Temp;

    MDK_Temp = MDKComputation(M, K);

    M = MDK_Temp.M_;
    D = MDK_Temp.D_;
    K = MDK_Temp.K_;

    // cout << "M^-1:" << M.inverse() << endl;
    cout << "D修改为:" << D << endl;
    cout << "K修改为:" << K << endl;
}

int main(int argc, char **argv)
{
#pragma region /*基本参数初始化*/
    ros::init(argc, argv, "admittance_control");

    ros::NodeHandle n;

    ros::Subscriber FTsensor_sub = n.subscribe("netft_data", 1, Force2Pose);
    ros::Subscriber servo_line_sub = n.subscribe("/robot_driver/tool_point", 1, ToolPointRecord);

    // zero_drift_compensation << -0.345442, -0.916142, -1.68673, -0.0594288, -0.0372112, -0.100919;
    // centroid_sensor << -0.00158413, -0.0023754, 0.00659773;
    // G_basis << 0.0, 0.0, -19.6636;
    zero_drift_compensation << -1.03534, -1.63669, -2.26417, -0.0383264, -0.00889498, -0.102886;
    centroid_sensor << 0.000226404, -4.35079e-05, 0.00441495;
    G_basis << 0.0, 0.0, -18.9807;

    origin_expected_pose << -0.698439031234, 0.00107985579317, 0.147448071114, -3.1372717645, 0.00903196524481, 0.00885404342115;
    transform_basis2end = AngleAxisd(origin_expected_pose(5, 0), Vector3d::UnitZ()) *
                          AngleAxisd(origin_expected_pose(4, 0), Vector3d::UnitY()) *
                          AngleAxisd(origin_expected_pose(3, 0), Vector3d::UnitX());
    homogeneous_transform_expected << transform_basis2end(0, 0), transform_basis2end(0, 1), transform_basis2end(0, 2), origin_expected_pose(0, 0),
        transform_basis2end(1, 0), transform_basis2end(1, 1), transform_basis2end(1, 2), origin_expected_pose(1, 0),
        transform_basis2end(2, 0), transform_basis2end(2, 1), transform_basis2end(2, 2), origin_expected_pose(2, 0),
        0, 0, 0, 1;
    origin_expected_joint << 0.16522547684382893, 2.026115249482023, 1.9502954394352305, 0.7277293271572154, -1.5764709940311654, 1.7271638412619614;

    expected_wrench << 0, 0, -5, 0, 0, 0;
    expected_pose << -0.699384694946, 0.0029545274708, 0.16970396014, 3.14058525409, 0.0026023751631, 0.0105711930739;

    transform_sensor2end = MatrixXd::Identity(6, 6);
    transform_sensor2end(3, 1) = -28.6 / 1000.0;
    transform_sensor2end(4, 0) = 28.6 / 1000.0;

    delta_pose = MatrixXd::Zero(6, 1);
    delta_pose_velocity = MatrixXd::Zero(6, 1);
    pre_delta_pose = MatrixXd::Zero(6, 1);

    // a d theta alpha
    MDH << 0.0, 119.87 / 1000, origin_expected_joint(0, 0), -0.13 / 180 * PI,
        0.0, 0.0, origin_expected_joint(1, 0), 90.00 / 180 * PI,
        555.24 / 1000, 0.0, origin_expected_joint(2, 0), 0.28 / 180 * PI,
        482.28 / 1000, -115.33 / 1000, origin_expected_joint(3, 0), 0.08 / 180 * PI,
        0.0, 113.23 / 1000, origin_expected_joint(4, 0), 90.01 / 180 * PI,
        0.0, 107.17 / 1000, origin_expected_joint(5, 0), -89.83 / 180 * PI;

    dynamic_parameter << 0, 0, 0, 0, 0, 9.7748, -3.4619, 0.0707, 0.3706, 0,
        0.2326, 7.9749, -1.6275, -0.0187, 0.2293, 0, -0.0997, 1.4314, 0.0172, 0.0085,
        0.0193, 0, 0.0333, 0.0301, 0.0081, 0.0051, 1.0000e-04, 0, -0.0127, 0.0154,
        0.0040, -1.0000e-03, -3.0000e-04, 0, 0.0026, 0.0066, -0.0295, 1.5549, 0, 0,
        5.8022, 0.0472, 0, 0, 2.4664, 0.0436, 0, 0, 0.0039, -0.2274,
        0, 0, 0.0027, 0.0486, 0, 0, 0.0016, 2.0000e-04, 0, 0;

    M_referance = MassMatrixComputation(MDH, dynamic_parameter);
    cout << M_referance << endl;

#pragma endregion

#pragma region /*Set the admittance parameter*/
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 6; j++)
            M(i, j) = M_referance(i, j);
    }
    cout << "M^-1:" << M.inverse() << endl;
    // M = MatrixXd::Identity(6, 6);
    // M(0, 0) = 10.0;
    // M(1, 1) = 10.0;
    // M(2, 2) = 10.0;
    D = MatrixXd::Identity(6, 6);
    K = MatrixXd::Identity(6, 6);

    /*动态调参服务*/
    dynamic_reconfigure::Server<admittance_control::reconfigureConfig> server;
    dynamic_reconfigure::Server<admittance_control::reconfigureConfig>::CallbackType Callback;
    Callback = boost::bind(&CallbackFunc, _1, _2);
    server.setCallback(Callback);
#pragma endregion

#pragma region /*Reach the Start Pose*/
    ros::ServiceClient client = n.serviceClient<robot_msgs::Move>("/robot_driver/move_line");
    ros::ServiceClient client_stop = n.serviceClient<std_srvs::Empty>("/robot_driver/stop_move");
    std_srvs::Empty srv_stop;

    srv.request.pose.clear();
    srv.request.is_block = true;
    srv.request.mvvelo = 0.1;
    srv.request.mvacc = 0.1;
    srv.request.pose.push_back(expected_pose(0, 0));
    srv.request.pose.push_back(expected_pose(1, 0));
    srv.request.pose.push_back(expected_pose(2, 0));
    srv.request.pose.push_back(expected_pose(3, 0));
    srv.request.pose.push_back(expected_pose(4, 0));
    srv.request.pose.push_back(expected_pose(5, 0));

    if (client.call(srv))
    {
        ROS_INFO("Response from server message: %s", srv.response.message.c_str());

        srv.request.pose.clear();
        srv.request.is_block = true;
        srv.request.mvvelo = 0.1;
        srv.request.mvacc = 0.1;
        srv.request.pose.push_back(origin_expected_pose(0, 0));
        srv.request.pose.push_back(origin_expected_pose(1, 0));
        srv.request.pose.push_back(origin_expected_pose(2, 0));
        srv.request.pose.push_back(origin_expected_pose(3, 0));
        srv.request.pose.push_back(origin_expected_pose(4, 0));
        srv.request.pose.push_back(origin_expected_pose(5, 0));
        if (client.call(srv))
        {
            ROS_INFO("Response from server message: %s", srv.response.message.c_str());
            ROS_INFO("Reach the Start Pose");
        }
        else
        {
            ROS_ERROR("failed to call /robot_driver/move_line");
            return 1;
        }
    }
#pragma endregion

    while (1)
    {
        /*更新外力，末端位置，导纳控制参数*/
        sleep(1);
        ros::spinOnce();

        // /*Debug*/
        delta_pose = MatrixXd::Zero(6, 1);
        delta_pose_velocity = MatrixXd::Zero(6, 1);
        pre_delta_pose = MatrixXd::Zero(6, 1);

        // delta_rotation << delta_homogeneous_transform.block<3, 3>(0, 0);
        // delta_angular = R2rpy(delta_rotation);
        // delta_pose << delta_homogeneous_transform.block<3, 1>(0, 3), delta_angular;

        /*计算传感器外力*/
        G_sensor = transform_basis2end.transpose() * G_basis;
        M_sensor = SkewSymmetry(centroid_sensor) * G_sensor;
        gravity_compensation << G_sensor, M_sensor;
        external_wrench_sensor = FTsensor_actual_data - gravity_compensation;
        cout << "外力：" << external_wrench_sensor(0, 0) << "," << external_wrench_sensor(1, 0) << "," << external_wrench_sensor(2, 0);
        cout << "外力矩：" << external_wrench_sensor(3, 0) << "," << external_wrench_sensor(4, 0) << "," << external_wrench_sensor(5, 0) << endl;

        /*计算末端位置误差*/
        delta_homogeneous_transform = homogeneous_transform_current.inverse() * homogeneous_transform_expected;
        delta_rotation << delta_homogeneous_transform.block<3, 3>(0, 0);
        delta_angular = R2rpy(delta_rotation);
        delta_pose << delta_homogeneous_transform.block<3, 1>(0, 3), delta_angular;
        delta_pose(3, 0) = AngularPI(delta_pose(3, 0));
        delta_pose(4, 0) = AngularPI(delta_pose(4, 0));
        delta_pose(5, 0) = AngularPI(delta_pose(5, 0));
        delta_pose_velocity = (delta_pose - pre_delta_pose) / kcontrol_rate;

        external_wrench_end = transform_sensor2end * external_wrench_sensor;
        delta_wrench = expected_wrench - external_wrench_end;

        delta_pose_acceleration = M.inverse() * delta_wrench - M.inverse() * D * delta_pose_velocity - M.inverse() * K * delta_pose;
        cout << "delta_wrench:" << delta_wrench.transpose() << endl;
        cout << "delta_pose:" << delta_pose.transpose() << endl;
        cout << "delta_pose_velocity:" << delta_pose_velocity.transpose() << endl;
        cout << "delta_pose_acceleration:" << delta_pose_acceleration.transpose() << endl;

        pre_delta_pose = delta_pose;

        delta_pose = delta_pose + delta_pose_velocity * kcontrol_rate + delta_pose_acceleration * kcontrol_rate * kcontrol_rate;

        transform_basis2end = AngleAxisd(delta_pose(5, 0), Vector3d::UnitZ()) *
                              AngleAxisd(delta_pose(4, 0), Vector3d::UnitY()) *
                              AngleAxisd(delta_pose(3, 0), Vector3d::UnitX());
        delta_homogeneous_transform << transform_basis2end(0, 0), transform_basis2end(0, 1), transform_basis2end(0, 2), delta_pose(0, 0),
            transform_basis2end(1, 0), transform_basis2end(1, 1), transform_basis2end(1, 2), delta_pose(1, 0),
            transform_basis2end(2, 0), transform_basis2end(2, 1), transform_basis2end(2, 2), delta_pose(2, 0),
            0, 0, 0, 1;
        delta_homogeneous_transform = homogeneous_transform_expected * delta_homogeneous_transform.inverse();
        delta_rotation << delta_homogeneous_transform.block<3, 3>(0, 0);
        delta_angular = R2rpy(delta_rotation);
        expected_pose << delta_homogeneous_transform.block<3, 1>(0, 3), delta_angular;

        expected_pose(3, 0) = AngularPI(expected_pose(3, 0));
        expected_pose(4, 0) = AngularPI(expected_pose(4, 0));
        expected_pose(5, 0) = AngularPI(expected_pose(5, 0));

        temp3 = delta_pose - pre_delta_pose;
        cout << "本次步进量：" << temp3.transpose() << endl;
        cout << "delta_pose:" << delta_pose.transpose() << endl;
        cout << "本次期望位置：" << expected_pose.transpose() << endl;

        external_force_sensor << external_wrench_sensor(0, 0), external_wrench_sensor(1, 0), external_wrench_sensor(2, 0);
        external_torque_sensor << external_wrench_sensor(3, 0), external_wrench_sensor(4, 0), external_wrench_sensor(5, 0);
        temp1 = external_force_sensor.array().abs();
        temp2 = external_torque_sensor.array().abs();

        // if (temp1.maxCoeff() > 50.0 || temp2.maxCoeff() > 5.0 ||
        //     abs(delta_pose(0, 0)) > 0.008 || abs(delta_pose(1, 0)) > 0.010 || abs(delta_pose(2, 0)) > 0.030 ||
        //     abs(delta_pose(3, 0)) > 5.0 / 180 * PI || abs(delta_pose(4, 0)) > 5.0 / 180 * PI || abs(delta_pose(5, 0)) > 3.0 / 180 * PI)
        // {
        //     if (client_stop.call(srv_stop))
        //     {
        //         ROS_INFO("Jog stop");
        //         return 0;
        //     }
        //     else
        //     {
        //         ROS_ERROR("failed to call /robot_driver/stop_move");
        //         return 1;
        //     }
        // }

        // srv.request.pose.clear();
        // srv.request.is_block = false;
        // srv.request.mvvelo = 0.1;
        // srv.request.mvacc = 0.1;
        // srv.request.pose.push_back(expected_pose(0, 0));
        // srv.request.pose.push_back(expected_pose(1, 0));
        // srv.request.pose.push_back(expected_pose(2, 0));
        // srv.request.pose.push_back(expected_pose(3, 0));
        // srv.request.pose.push_back(expected_pose(4, 0));
        // srv.request.pose.push_back(expected_pose(5, 0));

        // if (client.call(srv))
        // {
        //     ROS_INFO("Response from server message: %s", srv.response.message.c_str());
        // }
        // else
        // {
        //     ROS_ERROR("failed to call /robot_driver/move_line");
        //     return 1;
        // }
        // sleep(kcontrol_rate);
    }

    return 0;
}
