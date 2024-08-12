#include "ros/ros.h"
#include <cmath>
#include "std_msgs/String.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/unsupported/MatrixFunctions"
#include "admittance_control/reconfigureConfig.h"
#include "dynamic_reconfigure/server.h"
#include "dynamic_model.h"
#include "admittance_control/MDK_msg.h"
#include "admittance_control/Plot.h"

using namespace std;
using namespace Eigen;

const double PI = 3.1415926;

void CallbackFunc(admittance_control::reconfigureConfig &ConfigType_obj, uint32_t level, MatrixXd &M, MatrixXd &D, MatrixXd &K, ros::Publisher &MDK_publisher, admittance_control::MDK_msg &MDK)
{
    switch (level)
    {
    case 0:
        M(0, 0) = ConfigType_obj.M_F_x;
        break;
    case 1:
        M(1, 1) = ConfigType_obj.M_F_y;
        break;
    case 2:
        M(2, 2) = ConfigType_obj.M_F_z;
        break;
    case 3:
        M(3, 3) = ConfigType_obj.M_T_x;
        break;
    case 4:
        M(4, 4) = ConfigType_obj.M_T_y;
        break;
    case 5:
        M(5, 5) = ConfigType_obj.M_T_z;
        break;
    case 6:
        D(0, 0) = ConfigType_obj.D_F_x;
        break;
    case 7:
        D(1, 1) = ConfigType_obj.D_F_y;
        break;
    case 8:
        D(2, 2) = ConfigType_obj.D_F_z;
        break;
    case 9:
        D(3, 3) = ConfigType_obj.D_T_x;
        break;
    case 10:
        D(4, 4) = ConfigType_obj.D_T_y;
        break;
    case 11:
        D(5, 5) = ConfigType_obj.D_T_z;
        break;
    case 12:
        K(0, 0) = ConfigType_obj.K_F_x;
        break;
    case 13:
        K(1, 1) = ConfigType_obj.K_F_y;
        break;
    case 14:
        K(2, 2) = ConfigType_obj.K_F_z;
        break;
    case 15:
        K(3, 3) = ConfigType_obj.K_T_x;
        break;
    case 16:
        K(4, 4) = ConfigType_obj.K_T_y;
        break;
    case 17:
        K(5, 5) = ConfigType_obj.K_T_z;
        break;
    }

    // D = M * K;
    // D = 2 * D.sqrt();

    memcpy(&(MDK.M), M.data(), 8 * 36);
    memcpy(&(MDK.D), D.data(), 8 * 36);
    memcpy(&(MDK.K), K.data(), 8 * 36);

    MDK_publisher.publish(MDK);

    // cout << "M^-1:" << M.inverse() << endl;
    cout << level << endl;
    cout << "M修改为:" << M << endl;
    cout << "D修改为:" << D << endl;
    cout << "K修改为:" << K << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MDK_computation");

    ros::NodeHandle n;

    MatrixXd M(6, 6);
    MatrixXd D(6, 6);
    MatrixXd K(6, 6);

    MatrixXd MDH(6, 4);
    VectorXd dynamic_parameter(60);
    VectorXd expected_joint(6);

    admittance_control::MDK_msg MDK;

    ros::Publisher MDK_publisher = n.advertise<admittance_control::MDK_msg>("/MDK", 10);
    ros::Publisher plot_pub = n.advertise<admittance_control::Plot>("/plot_data", 100);

    /*计算末端质量矩阵*/
    expected_joint << 0.16522547684382893, 2.026115249482023, 1.9502954394352305, 0.7277293271572154, -1.5764709940311654, 1.7271638412619614;
    // a d theta alpha
    MDH << 0.0, 119.87 / 1000, expected_joint(0), -0.13 / 180 * PI,
        0.0, 0.0, expected_joint(1), 90.00 / 180 * PI,
        555.24 / 1000, 0.0, expected_joint(2), 0.28 / 180 * PI,
        482.28 / 1000, -115.33 / 1000, expected_joint(3), 0.08 / 180 * PI,
        0.0, 113.23 / 1000, expected_joint(4), 90.01 / 180 * PI,
        0.0, 107.17 / 1000, expected_joint(5), -89.83 / 180 * PI;

    dynamic_parameter << 0, 0, 0, 0, 0, 9.7748, -3.4619, 0.0707, 0.3706, 0,
        0.2326, 7.9749, -1.6275, -0.0187, 0.2293, 0, -0.0997, 1.4314, 0.0172, 0.0085,
        0.0193, 0, 0.0333, 0.0301, 0.0081, 0.0051, 1.0000e-04, 0, -0.0127, 0.0154,
        0.0040, -1.0000e-03, -3.0000e-04, 0, 0.0026, 0.0066, -0.0295, 1.5549, 0, 0,
        5.8022, 0.0472, 0, 0, 2.4664, 0.0436, 0, 0, 0.0039, -0.2274,
        0, 0, 0.0027, 0.0486, 0, 0, 0.0016, 2.0000e-04, 0, 0;

    // M = MassMatrixComputation(MDH, dynamic_parameter);
    M = MatrixXd::Identity(6, 6);
    D = MatrixXd::Identity(6, 6);
    K = MatrixXd::Identity(6, 6);

    double M_array[6] = {500, 500, 500, 7.5, 5, 30};
    double D_array[6] = {4000, 4000, 4000, 7.5, 5, 30};
    double K_array[6] = {0, 0, 0, 0, 0, 0};

    for (int i = 0; i < 6; i++)
    {
        M(i, i) = M_array[i];
        K(i, i) = K_array[i];
        // D(i, i) = 2 * sqrt(M_array[i] * K_array[i]);
        D(i, i) = D_array[i];
    }

    // D = M * K;
    // D = 2 * D.sqrt();

    memcpy(&(MDK.M), M.data(), 8 * 36);
    memcpy(&(MDK.D), D.data(), 8 * 36);
    memcpy(&(MDK.K), K.data(), 8 * 36);

    MDK_publisher.publish(MDK);

    /*动态调参服务*/
    dynamic_reconfigure::Server<admittance_control::reconfigureConfig> server;
    dynamic_reconfigure::Server<admittance_control::reconfigureConfig>::CallbackType Callback;
    Callback = boost::bind(&CallbackFunc, _1, _2, M, D, K, MDK_publisher, MDK);
    server.setCallback(Callback);

    // cout << "M^-1:" << M.inverse() << endl;

    ros::spin();

    return 0;
}