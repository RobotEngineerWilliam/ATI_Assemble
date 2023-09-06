#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include <cmath>
#include <queue>
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include "robot_msgs/Move.h"
#include "robot_msgs/ServoL.h"
#include "robot_msgs/GetPosition.h"
#include "admittance_control/MDK_msg.h"
#include "admittance_control/Plot.h"

using namespace std;
using namespace Eigen;

const double PI = 3.1415926;

MatrixXd M(6, 6);
MatrixXd D(6, 6);
MatrixXd K(6, 6);

pthread_mutex_t mutex;
VectorXd FTsensor_data(6);
VectorXd FTdata_sum(6);
queue<VectorXd> FTdata_list;
long FTdata_num = 10;
bool start_calibration = false;
long zero_drift_calibration_num = 0;
VectorXd zero_drift_calibration_sum(6);

Matrix3d rotation_basis2end = Matrix3d::Identity();
Matrix4d homogeneous_transform_current = Matrix4d::Identity();
admittance_control::Plot plot_data;

double AngularPI(double angular)
{
    if (abs(angular) > PI)
        angular = angular - angular / abs(angular) * 2 * PI;

    return angular;
}

static Matrix4d Pose2HomogeneousTransform(VectorXd pose)
{
    Matrix4d homogeneous_transform = Matrix4d::Identity();
    Matrix3d rotation_transform;

    rotation_transform = AngleAxisd(pose(5), Vector3d::UnitZ()) *
                         AngleAxisd(pose(4), Vector3d::UnitY()) *
                         AngleAxisd(pose(3), Vector3d::UnitX());
    homogeneous_transform.block<3, 3>(0, 0) = rotation_transform;
    homogeneous_transform.block<3, 1>(0, 3) = pose.block<3, 1>(0, 0);

    return homogeneous_transform;
}

static VectorXd HomogeneousTransform2Pose(Matrix4d homogeneous_transform)
{
    VectorXd pose(6);
    Matrix3d rotation_transform;

    pose.block<3, 1>(0, 0) = homogeneous_transform.block<3, 1>(0, 3);

    rotation_transform = homogeneous_transform.block<3, 3>(0, 0);

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

void ForceRecord(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
    VectorXd FTdata_once(6);

    FTdata_once(0) = msg->wrench.force.x;
    FTdata_once(1) = msg->wrench.force.y;
    FTdata_once(2) = msg->wrench.force.z;
    FTdata_once(3) = msg->wrench.torque.x;
    FTdata_once(4) = msg->wrench.torque.y;
    FTdata_once(5) = msg->wrench.torque.z;

    FTdata_list.push(FTdata_once);

    if (FTdata_list.size() > FTdata_num)
    {
        FTdata_sum = FTdata_sum - FTdata_list.front() + FTdata_once;
        FTdata_list.pop();
    }
    else
        FTdata_sum = FTdata_sum + FTdata_once;

    pthread_mutex_lock(&mutex);
    FTsensor_data = FTdata_sum / FTdata_list.size();
    pthread_mutex_unlock(&mutex);

    if (start_calibration)
    {
        zero_drift_calibration_sum = zero_drift_calibration_sum + FTsensor_data;
        zero_drift_calibration_num = zero_drift_calibration_num + 1;
    }

    // cout << setw(26) << left << "get FTsensor data" << FTsensor_data.transpose() << endl;
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

void *FTsensorFilter(void *args)
{
    ros::NodeHandle *n = (ros::NodeHandle *)args;

    ros::Subscriber MDK_sub = n->subscribe<admittance_control::MDK_msg>("/MDK", 1, &MDKRecord);
    // 如何初始化MDK参数即先收到一次topic
    ros::Subscriber FTsensor_sub = n->subscribe<geometry_msgs::WrenchStamped>("netft_data", 1, &ForceRecord);

    ros::spin();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "admittance_control");
    ros::NodeHandle n;

    pthread_mutex_init(&mutex, NULL);

#pragma region /*基本参数定义*/
    double kcontrol_rate = 0.1;

    /* FTsensor calibration */
    VectorXd zero_drift_compensation(6);
    Vector3d G_basis; // 基坐标系重力
    Vector3d centroid_sensor;
    Vector3d G_sensor = Vector3d::Zero(); // 传感器坐标系重力
    VectorXd gravity_compensation(6);
    VectorXd external_wrench_sensor(6);
    Vector3d external_force_sensor = Vector3d::Zero();
    Vector3d external_torque_sensor = Vector3d::Zero();
    MatrixXd jacobian_sensor2end(6, 6);
    VectorXd external_wrench_end(6);

    /*Admittance computation*/
    VectorXd expected_wrench(6);
    VectorXd delta_wrench(6);
    VectorXd expected_pose(6);
    Matrix4d expected_homogeneous_transform = Matrix4d::Zero();
    VectorXd pre_delta_pose(6);
    Matrix4d pre_homogeneous_transform = Matrix4d::Zero();
    VectorXd delta_pose(6);
    Matrix4d delta_homogeneous_transform = Matrix4d::Zero();
    VectorXd delta_pose_velocity(6);
    VectorXd delta_pose_acceleration(6);
    Vector3d current_postion;

    robot_msgs::ServoL servo_msg;

    cout.precision(4);
    // cout.setf(ios::scientific);
#pragma endregion

#pragma region /*基本参数初始化*/

    M = MatrixXd::Identity(6, 6);
    D = MatrixXd::Identity(6, 6);
    K = MatrixXd::Identity(6, 6);

    double M_array[6] = {100, 100, 150, 1, 1, 20};
    double D_array[6] = {500, 500, 500, 20, 20, 50};
    double K_array[6] = {80, 100, 200, 10, 10, 50};

    for (int i = 0; i < 6; i++)
    {
        M(i, i) = M_array[i];
        K(i, i) = K_array[i];
        // D(i, i) = 2 * sqrt(M_array[i] * K_array[i]);
        D(i, i) = D_array[i];
    }
    cout << "M修改为:" << M << endl;
    cout << "D修改为:" << D << endl;
    cout << "K修改为:" << K << endl;

    // zero_drift_compensation << -1.03534, -1.63669, -2.26417, -0.0383264, -0.00889498, -0.102886;
    zero_drift_compensation << -5.49, -2.99, -0.21, -0.393, 0.172, -0.157;
    centroid_sensor << 0.000226404, -4.35079e-05, 0.00441495;
    G_basis << 0.0, 0.0, -18.9807;
    // G_basis << 0.0, 0.0, 0.0;
    expected_wrench << 0, 0, -5, 0, 0, 0;
    expected_pose << -0.699384694946, 0.0029545274708, 0.16970396014, 3.14058525409, 0.0026023751631, 0.0105711930739;
    // expected_pose << -0.699384694946, 0.0029545274708, 0.35, 3.14058525409, 0.0026023751631, 0.0105711930739;
    FTsensor_data = VectorXd::Zero(6);
    gravity_compensation = VectorXd::Zero(6);
    external_wrench_sensor = VectorXd::Zero(6);
    jacobian_sensor2end = MatrixXd::Identity(6, 6);
    jacobian_sensor2end(3, 1) = -28.6 / 1000.0;
    jacobian_sensor2end(4, 0) = 28.6 / 1000.0;
    external_wrench_end = VectorXd::Zero(6);
    pre_delta_pose = VectorXd::Zero(6);
    delta_wrench = VectorXd::Zero(6);
    delta_pose = VectorXd::Zero(6);
    delta_pose_velocity = VectorXd::Zero(6);
    delta_pose_acceleration = VectorXd::Zero(6);

    FTdata_sum = VectorXd::Zero(6);
    zero_drift_calibration_sum = VectorXd::Zero(6);

    pthread_t tids_1;
    pthread_create(&tids_1, NULL, FTsensorFilter, &n);

    ros::Publisher servo_move_pub = n.advertise<robot_msgs::ServoL>("/robot_driver/servo_move", 1);
    ros::Publisher plot_pub = n.advertise<admittance_control::Plot>("/plot_data", 100);
    ros::ServiceClient client_get_position = n.serviceClient<robot_msgs::GetPosition>("/robot_driver/update_position");
    robot_msgs::GetPosition srv_get_position;

#pragma endregion

#pragma region /*Reach the Start Pose*/
    ros::ServiceClient client = n.serviceClient<robot_msgs::Move>("/robot_driver/move_line");
    robot_msgs::Move srv;
    ros::ServiceClient client_stop = n.serviceClient<std_srvs::Empty>("/robot_driver/stop_move");
    std_srvs::Empty srv_stop;

    srv.request.pose.clear();
    srv.request.is_block = true;
    srv.request.mvvelo = 0.1;
    srv.request.mvacc = 0.1;
    for (int i = 0; i < 6; i++)
        srv.request.pose.push_back(expected_pose(i));

    if (client.call(srv))
    {
        expected_pose << -0.698439031234, 0.00107985579317, 0.147448071114, -3.1372717645, 0.00903196524481, 0.00885404342115;
        // expected_pose << -0.698439031234, 0.00107985579317, 0.35, -3.1372717645, 0.0203196524481, 0.0285404342115;
        srv.request.pose.clear();
        srv.request.is_block = true;
        srv.request.mvvelo = 0.1;
        srv.request.mvacc = 0.1;
        for (int i = 0; i < 6; i++)
            srv.request.pose.push_back(expected_pose(i));

        if (client.call(srv))
            ROS_INFO("Reach the Start Pose");
        else
        {
            ROS_ERROR("Failed to Reach the Start Pose 2");
            return 1;
        }
    }
    else
    {
        ROS_ERROR("Failed to Reach the Start Pose 1");
        return 1;
    }
    start_calibration = true;

    pre_homogeneous_transform = Pose2HomogeneousTransform(expected_pose);

    sleep(5);

    // start_calibration = false;
    // zero_drift_compensation = zero_drift_calibration_sum / zero_drift_calibration_num;

    // cout << zero_drift_compensation.transpose() << endl;

    ros::Rate rate(10);
#pragma endregion

    while (ros::ok())
    {
        /*更新外力，末端位置，导纳控制参数*/
        // ros::spinOnce();

        srv_get_position.request.is_tcp_position = true;

        if (client_get_position.call(srv_get_position))
        {
            current_postion << srv_get_position.response.position[0], srv_get_position.response.position[1], srv_get_position.response.position[2];
            rotation_basis2end = AngleAxisd(srv_get_position.response.position[5], Vector3d::UnitZ()) *
                                 AngleAxisd(srv_get_position.response.position[4], Vector3d::UnitY()) *
                                 AngleAxisd(srv_get_position.response.position[3], Vector3d::UnitX());
            homogeneous_transform_current = Matrix4d::Identity();
            homogeneous_transform_current.block<3, 3>(0, 0) = rotation_basis2end;
            homogeneous_transform_current.block<3, 1>(0, 3) = current_postion;
            cout << setw(26) << left << "get tool point:" << current_postion.transpose() << endl;
            plot_data.data_13 = current_postion(0);
            plot_data.data_14 = current_postion(1);
            plot_data.data_15 = current_postion(2);
            plot_data.data_16 = srv_get_position.response.position[3];
            plot_data.data_17 = srv_get_position.response.position[4];
            plot_data.data_18 = srv_get_position.response.position[5];
        }
        else
        {
            ROS_ERROR("Failed to Get the Position");
            return 1;
        }
        /*Debug*/
        // delta_pose = MatrixXd::Zero(6, 1);
        // delta_pose_velocity = MatrixXd::Zero(6, 1);
        // pre_delta_pose = MatrixXd::Zero(6, 1);

        /*计算传感器外力*/
        G_sensor = rotation_basis2end.transpose() * G_basis;
        gravity_compensation << G_sensor, centroid_sensor.cross(G_sensor);

        pthread_mutex_lock(&mutex);
        FTsensor_data = FTsensor_data - zero_drift_compensation;
        external_wrench_sensor = FTsensor_data - gravity_compensation;
        pthread_mutex_unlock(&mutex);

        delta_wrench = external_wrench_sensor - expected_wrench;
        delta_wrench = jacobian_sensor2end * delta_wrench;

        // delta_wrench(2) = 5.0;
        cout << setw(26) << left << "external_wrench:" << external_wrench_sensor.transpose() << endl;
        cout << setw(26) << left << "delta_wrench:" << delta_wrench.transpose() << endl;

        /*计算xt,dotxt,dotdotxt*/
        delta_homogeneous_transform = homogeneous_transform_current.inverse() * pre_homogeneous_transform;
        delta_pose = -HomogeneousTransform2Pose(delta_homogeneous_transform);
        delta_pose_velocity = (delta_pose - pre_delta_pose) / kcontrol_rate;
        delta_pose_acceleration = M.inverse() * (delta_wrench - D * delta_pose_velocity - K * delta_pose);

        pre_homogeneous_transform = homogeneous_transform_current;
        pre_delta_pose = delta_pose;

        cout << setw(26) << left << "delta_pose:" << delta_pose.transpose() << endl;
        cout << setw(26) << left << "delta_pose_velocity:" << delta_pose_velocity.transpose() << endl;
        cout << setw(26) << left << "delta_pose_acceleration:" << delta_pose_acceleration.transpose() << endl;

        /*计算xt+1,期望位姿*/
        delta_pose = delta_pose + delta_pose_velocity * kcontrol_rate + delta_pose_acceleration * kcontrol_rate * kcontrol_rate;
        delta_homogeneous_transform = Pose2HomogeneousTransform(delta_pose);
        expected_homogeneous_transform = homogeneous_transform_current * delta_homogeneous_transform;
        expected_pose = HomogeneousTransform2Pose(expected_homogeneous_transform);

        cout << setw(32) << left << "本次步进量：" << delta_pose.transpose() << endl;
        cout << setw(33) << left << "本次期望位置：" << expected_pose.transpose() << endl;

        plot_data.data_1 = expected_pose(0);
        plot_data.data_2 = expected_pose(1);
        plot_data.data_3 = expected_pose(2);
        plot_data.data_4 = expected_pose(3);
        if (expected_pose(3) > 0)
            plot_data.data_4 = plot_data.data_4 - 2 * PI;
        plot_data.data_5 = expected_pose(4);
        plot_data.data_6 = expected_pose(5);
        plot_data.data_7 = external_wrench_sensor(0);
        plot_data.data_8 = external_wrench_sensor(1);
        plot_data.data_9 = external_wrench_sensor(2);
        plot_data.data_10 = external_wrench_sensor(3);
        plot_data.data_11 = external_wrench_sensor(4);
        plot_data.data_12 = external_wrench_sensor(5);
        plot_pub.publish(plot_data);

        /*接触力和位置限制*/
        external_force_sensor = external_wrench_sensor.block<3, 1>(0, 0);
        external_torque_sensor = external_wrench_sensor.block<3, 1>(3, 0);
        external_force_sensor = external_force_sensor.array().abs();
        external_torque_sensor = external_torque_sensor.array().abs();

        if (external_force_sensor.maxCoeff() > 100.0 || external_torque_sensor.maxCoeff() > 5.0 ||
            abs(expected_pose(0) + 0.698439031234) > 0.005 || abs(expected_pose(1) - 0.00107985579317) > 0.009 || abs(expected_pose(2) - 0.147448071114) > 0.02 ||
            abs(AngularPI(expected_pose(3) + PI)) > 3.0 / 180 * PI || abs(expected_pose(4)) > 3.0 / 180 * PI || abs(expected_pose(5)) > 5.0 / 180 * PI)
        {
            // if (client_stop.call(srv_stop))
            // {
            //     ROS_INFO("Jog stop");
            //     return 0;
            // }
            // else
            // {
            //     ROS_ERROR("failed to call /robot_driver/stop_move");
            //     return 1;
            // }
            ROS_INFO("Jog stop");
            servo_msg.servo_mode = false;
            servo_move_pub.publish(servo_msg);
            break;
        }

        servo_msg.servo_mode = true;
        for (int i = 0; i < 6; i++)
            servo_msg.pose[i] = expected_pose(i);
        servo_move_pub.publish(servo_msg);

        rate.sleep();
    }
    servo_msg.servo_mode = false;
    servo_move_pub.publish(servo_msg);

    pthread_join(tids_1, NULL);
    pthread_mutex_destroy(&mutex);

    return 0;
}
