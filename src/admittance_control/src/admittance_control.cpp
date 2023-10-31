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

using namespace std;
using namespace Eigen;

const double PI = 3.1415926;

pthread_mutex_t FTsensor_mutex;

MatrixXd M(6, 6);
MatrixXd D(6, 6);
MatrixXd K(6, 6);

VectorXd FTsensor_data(6);
VectorXd FTdata_sum(6);
queue<VectorXd> FTdata_list;
long FTdata_num = 10;

VectorXd refer_pose(6);

Matrix3d rotation_basis2end = Matrix3d::Identity();
Matrix4d T_current = Matrix4d::Identity();

admittance_control::Plot plot_data;

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

    pthread_mutex_lock(&FTsensor_mutex);
    FTsensor_data = FTdata_sum / FTdata_list.size();
    pthread_mutex_unlock(&FTsensor_mutex);
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

void ReferPoseCalculate(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    Matrix4d T_camera2base;
    Matrix4d T_marker2camera;
    Matrix4d T_hole2marker;
    Matrix4d T_tcp2hole;
    Matrix4d T_tcp2base;
    Matrix4d T_hole2base;
    Quaterniond orientation_camera(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);

    T_camera2base << 0.0118395, 0.999707, -0.0211567, -0.551863903476,
        0.999917, -0.0117279, 0.00543987, -0.0450592850083,
        0.00519016, -0.0212194, -0.999762, 1.03177476774,
        0, 0, 0, 1;

    T_marker2camera = Matrix4d::Identity();
    T_marker2camera.block<3, 3>(0, 0) = orientation_camera.toRotationMatrix() * AngleAxisd(PI, Vector3d::UnitX());
    T_marker2camera.block<3, 1>(0, 3) << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

    T_hole2marker = Matrix4d::Identity();
    T_hole2marker.block<3, 1>(0, 3) << 0.005, -0.077 / 2 - 0.002 - 0.22 - 0.065 / 2, 0;

    T_tcp2hole = Matrix4d::Identity();
    T_tcp2hole.block<3, 1>(0, 3) << -0.025, 0.43, -0.1616;

    T_hole2base = T_camera2base * T_marker2camera * T_hole2marker;
    T_tcp2base = T_hole2base * T_tcp2hole;

    refer_pose = HomogeneousTransform2Pose(T_hole2base);

    cout << refer_pose.transpose() << endl;

    refer_pose = HomogeneousTransform2Pose(T_tcp2base);

    cout << refer_pose.transpose() << endl;
}

void *FTsensorFilter(void *args)
{
    ros::NodeHandle n_param;

    ros::Subscriber MDK_sub = n_param.subscribe<admittance_control::MDK_msg>("/MDK", 1, &MDKRecord);
    // 如何初始化MDK参数即先收到一次topic
    ros::Subscriber FTsensor_sub = n_param.subscribe<geometry_msgs::WrenchStamped>("netft_data", 1, &ForceRecord);

    FTdata_sum = VectorXd::Zero(6);

    ros::spin();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "admittance_control");
    ros::NodeHandle n;

    pthread_mutex_init(&FTsensor_mutex, NULL);

    ros::CallbackQueue param_queue;
    n.setCallbackQueue(&param_queue);

    ros::Subscriber camera_pose = n.subscribe<geometry_msgs::PoseStamped>("/aruco_single/pose", 10, ReferPoseCalculate);

    sleep(2);

    param_queue.callOne(ros::WallDuration(1.0));

#pragma region /*基本参数定义*/
    double kcontrol_rate = 0.1;

    /* FTsensor CSalibration */
    VectorXd zero_drift_compensation(6);
    Vector3d G_basis; // 基坐标系重力
    Vector3d centroid_sensor;
    Vector3d G_sensor = Vector3d::Zero(); // 传感器坐标系重力
    VectorXd gravity_compensation(6);
    VectorXd external_wrench_sensor(6);
    Vector3d external_force_sensor = Vector3d::Zero();
    Vector3d external_torque_sensor = Vector3d::Zero();

    /* Admittance Computation */
    MatrixXd jacobian_sensor2peg(6, 6);
    Matrix4d T_peg2tcp;
    VectorXd expected_wrench(6);
    VectorXd delta_wrench(6);
    VectorXd expected_pose(6);
    Matrix4d T_expected = Matrix4d::Zero();
    VectorXd pre_delta_pose(6);
    Matrix4d T_pre = Matrix4d::Zero();
    VectorXd delta_pose(6);
    Matrix4d T_delta = Matrix4d::Zero();
    VectorXd delta_pose_velocity(6);
    VectorXd delta_pose_acceleration(6);
    Vector3d current_postion;

    /* Protection */
    double force_limitation = 100.0;
    double torque_limitation = 5.0;
    VectorXd pose_referrance(6);
    VectorXd pose_limitation(6);
    Vector3d delta_force = Vector3d::Zero();
    Vector3d delta_force_abs = Vector3d::Zero();
    Vector3d delta_torque = Vector3d::Zero();
    Vector3d delta_torque_abs = Vector3d::Zero();

    /* 状态切换 */
    long reach_count = 0;
    int stage = 1; // 一阶段导纳控制单侧入孔、二阶段另一侧入孔判断及搜孔、三阶段导纳控制完成竖直装配，四阶段完成水平装配
    int search_stage = 1;
    int step_count = 0;
    bool is_finite = true;
    int tra_num = 0;
    long external_wrench_num = 10;
    queue<VectorXd> external_wrench_list;
    Matrix4d T_pegright2base;
    Matrix4d T_pegright2tcp;
    Matrix3d R_calc;
    VectorXd start_pose(6);
    VectorXd change_wrench(6);
    VectorXd pre_change_wrench(6);
    VectorXd inspect_pose(6);
    MatrixXd tra2start_pose(6, 300);
    MatrixXd tra2inspected_pose(6, 300);

    robot_msgs::ServoL servo_msg;

    cout.precision(4);
    // cout.setf(ios::scientific);
#pragma endregion

#pragma region /*基本参数初始化*/

    M = MatrixXd::Identity(6, 6);
    D = MatrixXd::Identity(6, 6);
    K = MatrixXd::Identity(6, 6);

    double M_array[6] = {500, 500, 500, 5, 5, 5};
    double D_array[6] = {250, 250, 250, 5, 5, 5};
    double K_array[6] = {0, 0, 0, 0, 0, 0};

    for (int i = 0; i < 6; i++)
    {
        M(i, i) = M_array[i];
        K(i, i) = K_array[i];
        D(i, i) = D_array[i];
    }
    cout << "M修改为:" << M << endl;
    cout << "D修改为:" << D << endl;
    cout << "K修改为:" << K << endl;

    /* FTsensor Calibration */
    zero_drift_compensation << -4.5787, -0.881606, -1.33029, -0.354722, -0.0634506, -0.145445;
    centroid_sensor << -0.00975944, 0.00113186, 0.0058718;
    G_basis << 0.0, 0.0, -19.0263;
    FTsensor_data = VectorXd::Zero(6);
    gravity_compensation = VectorXd::Zero(6);
    external_wrench_sensor = VectorXd::Zero(6);
    jacobian_sensor2peg = MatrixXd::Identity(6, 6);
    jacobian_sensor2peg(3, 1) = 43 / 1000.0;
    jacobian_sensor2peg(4, 0) = -43 / 1000.0;

    /* Admittance Computation */
    T_pegright2tcp = Matrix4d::Identity();
    T_pegright2tcp(2, 3) = 111.6 / 1000.0;
    T_pegright2tcp(1, 3) = 0.4;
    T_peg2tcp = Matrix4d::Identity();
    T_peg2tcp(2, 3) = 111.6 / 1000.0;
    expected_wrench << -20, 20, -50, 0, 0, 0;
    expected_pose = refer_pose;
    T_pre = Pose2HomogeneousTransform(expected_pose);
    pre_delta_pose = VectorXd::Zero(6);
    delta_wrench = VectorXd::Zero(6);
    delta_pose = VectorXd::Zero(6);
    delta_pose_velocity = VectorXd::Zero(6);
    delta_pose_acceleration = VectorXd::Zero(6);

    /* Protection */
    pose_limitation << 0.02, 0.02, 0.3, 15.0 / 180 * PI, 15.0 / 180 * PI, 5.0 / 180 * PI;
    pose_referrance << -0.707, 0.023, 0.2, -PI, 0.0, 0.0;
    pre_change_wrench = VectorXd::Zero(6);

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
        ROS_INFO("Reach the Start Pose");
    else
    {
        ROS_ERROR("Failed to Reach the Start Pose 1");
        return 1;
    }
#pragma endregion

    ros::Rate rate(1 / kcontrol_rate);

    while (ros::ok())
    {
        /* 更新末端位置 */
        srv_get_position.request.is_tcp_position = true;

        if (client_get_position.call(srv_get_position))
        {
            current_postion << srv_get_position.response.position[0], srv_get_position.response.position[1], srv_get_position.response.position[2];
            rotation_basis2end = AngleAxisd(srv_get_position.response.position[5], Vector3d::UnitZ()) *
                                 AngleAxisd(srv_get_position.response.position[4], Vector3d::UnitY()) *
                                 AngleAxisd(srv_get_position.response.position[3], Vector3d::UnitX());
            T_current = Matrix4d::Identity();
            T_current.block<3, 3>(0, 0) = rotation_basis2end;
            T_current.block<3, 1>(0, 3) = current_postion;

            for (int i = 0; i < 6; i++)
                plot_data.data[i] = srv_get_position.response.position[i];
        }
        else
        {
            ROS_ERROR("Failed to Get the Position");
            return 1;
        }

        /* 计算传感器外力 */
        G_sensor = rotation_basis2end.transpose() * G_basis;
        gravity_compensation << G_sensor, centroid_sensor.cross(G_sensor);

        pre_change_wrench = change_wrench;

        pthread_mutex_lock(&FTsensor_mutex);
        FTsensor_data = FTsensor_data - zero_drift_compensation;
        external_wrench_sensor = FTsensor_data - gravity_compensation;
        pthread_mutex_unlock(&FTsensor_mutex);

        external_wrench_list.push(external_wrench_sensor);

        if (external_wrench_list.size() > external_wrench_num)
            external_wrench_list.pop();

        /* 计算工件坐标系下deltaF */
        delta_wrench = jacobian_sensor2peg * (external_wrench_sensor - expected_wrench);

        delta_force = delta_wrench.block<3, 1>(0, 0);
        delta_force_abs = delta_force.array().abs();
        delta_torque = delta_wrench.block<3, 1>(3, 0);
        delta_torque_abs = delta_torque.array().abs();

        /* 计算xt,dotxt,dotdotxt */
        T_pre = T_pre * T_peg2tcp;
        T_delta = T_pre.inverse() * T_current * T_peg2tcp;
        delta_pose = HomogeneousTransform2Pose(T_delta);
        delta_pose_velocity = (delta_pose - pre_delta_pose) / kcontrol_rate;
        delta_pose_acceleration = M.inverse() * (delta_wrench - D * delta_pose_velocity - K * delta_pose);

        T_pre = T_current;
        pre_delta_pose = delta_pose;

        /* 计算xt+1,期望位姿 */
        delta_pose = delta_pose + delta_pose_velocity * kcontrol_rate + delta_pose_acceleration * kcontrol_rate * kcontrol_rate;
        T_delta = Pose2HomogeneousTransform(delta_pose);
        T_expected = T_current * T_peg2tcp * T_delta * T_peg2tcp.inverse();
        expected_pose = HomogeneousTransform2Pose(T_expected);

        /* 数据观测 */
        for (int i = 0; i < 6; i++)
        {
            plot_data.data[i + 6] = expected_pose[i];
            plot_data.data[i + 12] = external_wrench_sensor[i];
            plot_data.data[i + 18] = delta_pose[i];
        }

        if (expected_pose(3) > 0)
            plot_data.data[3] = plot_data.data[3] - 2 * PI;

        /* 运动切换 */
        switch (stage)
        {
        case 1:
            if (delta_force_abs.maxCoeff() < 5 && delta_torque(0) < 1.0)
            {
                if (reach_count <= 10)
                    reach_count++;
                else
                {
                    ROS_INFO("Finish the first stage");

                    stage = 2;
                    start_pose = expected_pose;
                    tra2start_pose.block<6, 1>(0, 0) = expected_pose;
                    expected_wrench << 0, 0, -20, 0, 0, 0;
                    reach_count = 0;
                }
            }
            else
                reach_count = 0;

            break;

        case 2:
            /* 入孔状态判断 */
            T_pegright2base = T_expected * T_pegright2tcp;

            switch (search_stage)
            {
            case 0: // 区分孔内孔外
                break;
            case 1: // 顺时针搜索
                if (step_count < 200 && abs(external_wrench_sensor(2) - external_wrench_list.front()(2)) < 15)
                {
                    T_expected = Matrix4d::Identity();
                    R_calc = AngleAxisd(0.1 / 180 * PI, Vector3d::UnitZ());
                    T_expected.block<3, 3>(0, 0) = R_calc;
                    T_expected = T_pegright2base * T_expected * T_pegright2tcp.inverse();
                    expected_pose = HomogeneousTransform2Pose(T_expected);
                    tra2start_pose.block<6, 1>(0, step_count + 1) = expected_pose;

                    step_count = step_count + 1;
                }
                else if (abs(pre_change_wrench(2) + change_wrench(2)) > 6)
                {
                    if (is_finite)
                        search_stage = 2;
                    else
                    {
                        stage = 3;
                        expected_wrench << 0, 10, -50, 0, 0, 0;
                        step_count = 0;
                    }
                }
                else
                {
                    expected_pose = tra2start_pose.block<6, 1>(0, step_count - tra_num - 1);

                    tra_num = tra_num + 1;

                    if (tra_num >= step_count)
                    {
                        search_stage = 3;
                        step_count = 0;
                        tra_num = 0;
                    }
                }

                break;

            case 2:                   // 顺时针确认
                if (step_count < 205) // 下降一段距离
                {
                    inspect_pose = expected_pose;
                    tra2inspected_pose.block<6, 1>(0, 0) = inspect_pose;
                    tra2start_pose.block<6, 1>(0, step_count + 1) = expected_pose;
                    step_count = step_count + 1;
                }
                else
                {
                    if (step_count < 220)
                    {
                        T_pegright2base = T_current * T_pegright2tcp; // 加导纳还是不加需要实验
                        T_expected = Matrix4d::Identity();
                        R_calc = AngleAxisd(0.1 / 180 * PI, Vector3d::UnitZ());
                        T_expected.block<3, 3>(0, 0) = R_calc;
                        T_expected = T_pegright2base * T_expected * T_pegright2tcp.inverse();
                        expected_pose = HomogeneousTransform2Pose(T_expected);

                        tra2inspected_pose.block<6, 1>(0, step_count - 204) = expected_pose;
                        tra2start_pose.block<6, 1>(0, step_count + 1) = expected_pose;

                        step_count = step_count + 1;

                        if (abs(pre_change_wrench(0) + change_wrench(0)) > 6 ||
                            abs(pre_change_wrench(1) + change_wrench(1)) > 6 ||
                            abs(pre_change_wrench(2) + change_wrench(2)) > 6) // 检测碰撞
                            search_stage = 5;
                    }
                    else
                    {
                        expected_pose = tra2start_pose.block<6, 1>(0, step_count - tra_num - 1);

                        tra_num = tra_num + 1;

                        if (tra_num >= step_count)
                        {
                            search_stage = 3;
                            step_count = 0;
                            tra_num = 0;
                        }
                    }
                }

                break;

            case 3: // 逆时针搜索
                if (step_count < 200 && abs(pre_change_wrench(2) + change_wrench(2)) < 6)
                {
                    T_expected = Matrix4d::Identity();
                    R_calc = AngleAxisd(-0.1 / 180 * PI, Vector3d::UnitZ());
                    T_expected.block<3, 3>(0, 0) = R_calc;
                    T_expected = T_pegright2base * T_expected * T_pegright2tcp.inverse();
                    expected_pose = HomogeneousTransform2Pose(T_expected);
                    tra2start_pose.block<6, 1>(0, step_count + 1) = expected_pose;

                    step_count = step_count + 1;
                }
                else if (abs(pre_change_wrench(2) + change_wrench(2)) > 6)
                {
                    if (is_finite)
                        search_stage = 4;
                    else
                    {
                        stage = 3;
                        expected_wrench << 0, 10, -50, 0, 0, 0;
                        step_count = 0;
                    }
                }
                else
                    ROS_ERROR("Failed to Find the Hole");

                break;

            case 4:                   // 顺时针确认
                if (step_count < 205) // 下降一段距离
                {
                    inspect_pose = expected_pose;
                    tra2inspected_pose.block<6, 1>(0, 0) = inspect_pose;
                    tra2start_pose.block<6, 1>(0, step_count + 1) = expected_pose;
                    step_count = step_count + 1;
                }
                else
                {
                    if (step_count < 220)
                    {
                        T_pegright2base = T_current * T_pegright2tcp; // 加导纳还是不加需要实验
                        T_expected = Matrix4d::Identity();
                        R_calc = AngleAxisd(-0.1 / 180 * PI, Vector3d::UnitZ());
                        T_expected.block<3, 3>(0, 0) = R_calc;
                        T_expected = T_pegright2base * T_expected * T_pegright2tcp.inverse();
                        expected_pose = HomogeneousTransform2Pose(T_expected);

                        tra2inspected_pose.block<6, 1>(0, step_count - 204) = expected_pose;
                        tra2start_pose.block<6, 1>(0, step_count + 1) = expected_pose;

                        step_count = step_count + 1;

                        if (abs(pre_change_wrench(0) + change_wrench(0)) > 6 ||
                            abs(pre_change_wrench(1) + change_wrench(1)) > 6 ||
                            abs(pre_change_wrench(2) + change_wrench(2)) > 6) // 检测碰撞
                            search_stage = 5;
                    }
                    else
                        ROS_ERROR("Failed to Find the Hole");
                }

                break;

            case 5: // return to inspected pose
                expected_pose = tra2inspected_pose.block<6, 1>(0, step_count - 206 - tra_num);

                tra_num = tra_num + 1;

                if (tra_num >= step_count - 205)
                {
                    stage = 3;
                    expected_wrench << 0, 10, -50, 0, 0, 0;
                    step_count = 0;
                    tra_num = 0;
                }
                break;
            }

            break;

        case 3:

            if (delta_force(2) < 0 && delta_torque.maxCoeff() < 0.2)
            {
                if (reach_count <= 10)
                    reach_count++;
                else
                {
                    stage = 4;
                    reach_count = 0;
                    expected_wrench << 0, -40, -5, 0, 0, 0;
                    pose_referrance << -0.698439031234, 0.00107985579317, 0.138448071114, -PI, 0.0, 0.0;
                    pose_limitation << 0.005, 0.11, 0.009, 3.0 / 180 * PI, 3.0 / 180 * PI, 5.0 / 180 * PI;
                }
            }
            else
                reach_count = 0;

            break;
        case 4:

            // expected_pose = HomogeneousTransform2Pose(T_current);

            break;
        }

        /*接触力和位置限制*/
        // external_force_sensor = external_wrench_sensor.block<3, 1>(0, 0);
        // external_torque_sensor = external_wrench_sensor.block<3, 1>(3, 0);
        // external_force_sensor = external_force_sensor.array().abs();
        // external_torque_sensor = external_torque_sensor.array().abs();

        // if (external_force_sensor.maxCoeff() > force_limitation || external_torque_sensor.maxCoeff() > torque_limitation ||
        //     abs(expected_pose(0) - pose_referrance(0)) > pose_limitation(0) || abs(expected_pose(1) - pose_referrance(1)) > pose_limitation(1) ||
        //     abs(expected_pose(2) - pose_referrance(2)) > pose_limitation(2) || abs(AngularPI(expected_pose(3) - pose_referrance(3))) > pose_limitation(3) ||
        //     abs(expected_pose(4) - pose_referrance(4)) > pose_limitation(4) || abs(expected_pose(5) - pose_referrance(5)) > pose_limitation(5))
        // {
        //     // if (client_stop.call(srv_stop))
        //     // {
        //     //     ROS_INFO("Jog stop");
        //     //     return 0;
        //     // }
        //     // else
        //     // {
        //     //     ROS_ERROR("failed to call /robot_driver/stop_move");
        //     //     return 1;
        //     // }
        //     ROS_INFO("Jog stop");
        //     servo_msg.servo_mode = false;
        //     servo_move_pub.publish(servo_msg);
        //     break;
        // }

        plot_pub.publish(plot_data);

        servo_msg.servo_mode = true;
        for (int i = 0; i < 6; i++)
            servo_msg.pose[i] = expected_pose(i);
        servo_move_pub.publish(servo_msg);

        rate.sleep();
    }
    servo_msg.servo_mode = false;
    servo_move_pub.publish(servo_msg);

    pthread_join(tids_1, NULL);
    pthread_mutex_destroy(&FTsensor_mutex);

    return 0;
}
