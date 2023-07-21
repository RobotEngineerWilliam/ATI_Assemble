#include "dynamic_model.h"

using namespace std;
using namespace Eigen;

const double PI = 3.1415926;

#pragma region /*Parameter for Mass Matrix Computation*/

#pragma endregion

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

MatrixXd LTransform(MatrixXd vector)
{
    MatrixXd cross_matrix = MatrixXd::Zero(3, 6);
    cross_matrix(0, 0) = vector(0, 0);
    cross_matrix(0, 1) = vector(1, 0);
    cross_matrix(0, 2) = vector(2, 0);
    cross_matrix(1, 1) = vector(0, 0);
    cross_matrix(1, 3) = vector(1, 0);
    cross_matrix(1, 4) = vector(2, 0);
    cross_matrix(2, 2) = vector(0, 0);
    cross_matrix(2, 4) = vector(1, 0);
    cross_matrix(2, 5) = vector(2, 0);

    return cross_matrix;
}

MatrixXd MassMatrixComputation(MatrixXd MDH, MatrixXd dynamic_parameter)
{
    MatrixXd acceleration_joint(3, 7);
    MatrixXd angular_acceleration_joint(3, 7);
    MatrixXd angular_acceleration(3, 6);
    MatrixXd homogeneous_transform_element(4, 4);
    Matrix<MatrixXd, 6, 6> homogeneous_transform;
    MatrixXd rotation_element(3, 3);
    Matrix<MatrixXd, 6, 6> rotation_joint;
    MatrixXd translation_element(3, 1);
    Matrix<MatrixXd, 6, 6> translation_joint;
    MatrixXd U_element(3, 3);
    Matrix<MatrixXd, 1, 6> U_temp;
    MatrixXd V_element(3, 6);
    Matrix<MatrixXd, 1, 6> V_temp;
    MatrixXd Y(6, 60);
    MatrixXd torque_G(6, 1);
    MatrixXd mass_matrix(6, 6);
    MatrixXd jacobi_joint2tool(6, 6);
    Matrix3d temp1;
    Matrix3d temp2;
    MatrixXd temp3(3, 1);

    /*forward kinematic*/
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 6; j++)
            homogeneous_transform(i, j) = MatrixXd::Identity(4, 4);
    }

    for (int i = 0; i < 6; i++)
    {
        homogeneous_transform_element << cos(MDH(i, 2)), -sin(MDH(i, 2)), 0, MDH(i, 0),
            sin(MDH(i, 2)) * cos(MDH(i, 3)), cos(MDH(i, 2)) * cos(MDH(i, 3)), -sin(MDH(i, 3)), -sin(MDH(i, 3)) * MDH(i, 1),
            sin(MDH(i, 2)) * sin(MDH(i, 3)), cos(MDH(i, 2)) * sin(MDH(i, 3)), cos(MDH(i, 3)), cos(MDH(i, 3)) * MDH(i, 1),
            0, 0, 0, 1;
        for (int j = 0; j < i + 1; j++)
        {
            for (int k = i; k < 6; k++)
                homogeneous_transform(k, j) = homogeneous_transform(k, j) * homogeneous_transform_element;
        }
    }
    cout << homogeneous_transform(5, 0) << endl;

    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            homogeneous_transform_element = homogeneous_transform(i, j);
            rotation_element << homogeneous_transform_element.block<3, 3>(0, 0);
            translation_element << homogeneous_transform_element.block<3, 1>(0, 3);
            rotation_joint(i, j) = rotation_element;
            translation_joint(i, j) = translation_element;
        }
    }

    angular_acceleration_joint.col(0) << 0, 0, 0;
    acceleration_joint.col(0) << 0, 0, -9.8;

    angular_acceleration = MatrixXd::Zero(3, 6);

    for (int i = 1; i < 7; i++)
    {
        rotation_element = rotation_joint(i - 1, i - 1);
        angular_acceleration_joint.col(i) = rotation_element.transpose() * angular_acceleration_joint.col(i - 1) + angular_acceleration.col(i - 1);
        acceleration_joint.col(i) = rotation_element.transpose() *
                                    (acceleration_joint.col(i - 1) +
                                     SkewSymmetry(angular_acceleration_joint.col(i - 1)) * translation_joint(i - 1, i - 1));
        U_element = SkewSymmetry(angular_acceleration_joint.col(i));
        U_temp(0, i - 1) = U_element;
        V_element = LTransform(angular_acceleration_joint.col(i));
        V_temp(0, i - 1) = V_element;
    }

    /*Coefficient Comput*/
    Y = MatrixXd::Zero(6, 60);

    for (int i = 0; i < 6; i++)
    {
        V_element = V_temp(0, i);
        U_element = U_temp(0, i);
        Y.block<1, 6>(i, 6 * i) << V_element.row(2);
        Y.block<1, 3>(i, 4 * i + 36) << temp1.row(2);
        if (i != 0)
        {
            temp3 = acceleration_joint.col(i + 1);
            temp1 = -SkewSymmetry(temp3);
            for (int j = 0; j < i; j++)
            {
                temp2 = SkewSymmetry(translation_joint(i - 1, j + 1));
                rotation_element = rotation_joint(i, j + 1);
                Y.block<1, 6>(j, 6 * i) << (rotation_element * V_element).row(2);
                Y.block<1, 3>(j, 4 * i + 32) << (rotation_element * temp1 + temp2 * rotation_element * U_element).row(2);
                Y.block<1, 1>(j, 4 * i + 35) = (temp2 * rotation_element * temp3).row(2);
            }
        }
    }

    torque_G = Y * dynamic_parameter;

    for (int q = 0; q < 6; q++)
    {
        angular_acceleration = MatrixXd::Zero(3, 6);
        angular_acceleration.col(q) << 0, 0, 1;

        for (int i = 1; i < 7; i++)
        {
            rotation_element = rotation_joint(i - 1, i - 1);
            angular_acceleration_joint.col(i) = rotation_element.transpose() * angular_acceleration_joint.col(i - 1) + angular_acceleration.col(i - 1);
            acceleration_joint.col(i) = rotation_element.transpose() *
                                        (acceleration_joint.col(i - 1) +
                                         SkewSymmetry(angular_acceleration_joint.col(i - 1)) * translation_joint(i - 1, i - 1));
            U_element = SkewSymmetry(angular_acceleration_joint.col(i));
            U_temp(i - 1) = U_element;
            V_element = LTransform(angular_acceleration_joint.col(i));
            V_temp(i - 1) = V_element;
        }

        /*Coefficient Comput*/
        Y = MatrixXd::Zero(6, 60);

        for (int i = 0; i < 6; i++)
        {
            V_element = V_temp(0, i);
            U_element = U_temp(0, i);
            Y.block<1, 6>(i, 6 * i) << V_element.row(2);
            Y.block<1, 3>(i, 4 * i + 36) << temp1.row(2);
            if (i != 0)
            {
                temp3 = acceleration_joint.col(i + 1);
                temp1 = -SkewSymmetry(temp3);
                for (int j = 0; j < i; j++)
                {
                    temp2 = SkewSymmetry(translation_joint(i - 1, j + 1));
                    rotation_element = rotation_joint(i, j + 1);
                    Y.block<1, 6>(j, 6 * i) << (rotation_element * V_element).row(2);
                    Y.block<1, 3>(j, 4 * i + 32) << (rotation_element * temp1 + temp2 * rotation_element * U_element).row(2);
                    Y.block<1, 1>(j, 4 * i + 35) = (temp2 * rotation_element * temp3).row(2);
                }
            }
        }

        mass_matrix.col(q) = Y * dynamic_parameter - torque_G;
    }

    jacobi_joint2tool = MatrixXd::Zero(6, 6);
    for (int i = 1; i < 6; i++)
    {
        homogeneous_transform_element = homogeneous_transform(5, i);
        jacobi_joint2tool(0, i - 1) = -homogeneous_transform_element(0, 0) * homogeneous_transform_element(1, 3) + homogeneous_transform_element(1, 0) * homogeneous_transform_element(0, 3);
        jacobi_joint2tool(1, i - 1) = -homogeneous_transform_element(0, 1) * homogeneous_transform_element(1, 3) + homogeneous_transform_element(1, 1) * homogeneous_transform_element(0, 3);
        jacobi_joint2tool(2, i - 1) = -homogeneous_transform_element(0, 2) * homogeneous_transform_element(1, 3) + homogeneous_transform_element(1, 2) * homogeneous_transform_element(0, 3);
        jacobi_joint2tool(3, i - 1) = homogeneous_transform_element(2, 0);
        jacobi_joint2tool(4, i - 1) = homogeneous_transform_element(2, 1);
        jacobi_joint2tool(5, i - 1) = homogeneous_transform_element(2, 2);
    }
    jacobi_joint2tool(5, 5) = 1;

    jacobi_joint2tool = jacobi_joint2tool.inverse();
    mass_matrix = jacobi_joint2tool.transpose() * mass_matrix * jacobi_joint2tool;

    return mass_matrix;
}

// #pragma region /*Parameter for Mass Matrix Computation*/
// MatrixXd MDH(6, 4);
// MatrixXd Vector_Joint2Centroid_Element(3, 1);
// Matrix<MatrixXd, 1, 6> Vector_Joint2Centroid;
// MatrixXd Mass_Matrix_Centroid_Element(6, 6);
// Matrix<MatrixXd, 1, 6> Mass_Matrix_Centroid;
// MatrixXd homogeneous_transform_element(4, 4);
// Matrix<MatrixXd, 6, 6> homogeneous_transform;
// MatrixXd rotation_element(3, 3);
// Matrix<MatrixXd, 6, 6> rotation_joint;
// MatrixXd translation_element(3, 1);
// Matrix<MatrixXd, 6, 6> translation_joint;
// MatrixXd Jacobi_Joint2Centroid_Element(6, 6);
// Matrix<MatrixXd, 1, 6> Jacobi_Joint2Centroid;
// MatrixXd jacobi_joint2tool(6, 6);
// MatrixXd Mass_Matrix_Joint_Element(6, 6);
// Matrix<MatrixXd, 1, 6> Mass_Matrix_Joint;
// MatrixXd mass_matrix(6, 6);
// #pragma endregion

// void MassMatrixComput(MatrixXd Pose)
// {
//     // a d theta alpha
//     MDH << 0.0, 119.87, Pose(0, 0), -0.13,
//         0.0, 0.0, Pose(1, 0), 90.00,
//         555.24, 0.0, Pose(2, 0), 0.28,
//         482.28, -115.33, Pose(3, 0), 0.08,
//         0.0, 113.23, Pose(4, 0), 90.01,
//         0.0, 107.17, Pose(5, 0), -89.83;

//     const double Ixx[6] = {0, -3.4619, -1.6275, 0.0172, 0.0081, 0.004};
//     const double Ixy[6] = {0, 0.0707, -0.0187, 0.0085, 0.0051, -0.001};
//     const double Ixz[6] = {0, 0.3706, 0.2293, 0.0193, 0.0001, -0.0003};
//     const double Iyy[6] = {0, 0, 0, 0, 0, 0};
//     const double Iyz[6] = {0, 0.2326, -0.0997, 0.0333, -0.0127, 0.0026};
//     const double Izz[6] = {9.7748, 7.9749, 1.4314, 0.0301, 0.0154, 0.0066};
//     const double m[6] = {-0.0295, 5.8022, 2.4664, 0.0039, 0.0027, 0.0016};
//     const double mx[6] = {1.5549, 0.0472, 0.0436, -0.2274, 0.0486, 0.0002};

//     for (int i = 0; i < 6; i++)
//     {
//         Mass_Matrix_Centroid_Element << m[i], 0, 0, 0, 0, 0,
//             0, m[i], 0, 0, 0, 0,
//             0, 0, m[i], 0, 0, 0,
//             0, 0, 0, Ixx[i], Ixy[i], Ixz[i],
//             0, 0, 0, Ixy[i], Iyy[i], Iyz[i],
//             0, 0, 0, Ixz[i], Iyz[i], Izz[i];
//         Mass_Matrix_Centroid(0, i) = Mass_Matrix_Centroid_Element;

//         Vector_Joint2Centroid_Element << mx[i] / m[i], 0, 0;
//         Vector_Joint2Centroid(0, i) = Vector_Joint2Centroid_Element;
//     }

//     for (int i = 0; i < 6; i++)
//     {
//         for (int j = 0; j < 6; j++)
//             homogeneous_transform(i, j) = MatrixXd::Identity(4, 4);
//     }

//     for (int i = 0; i < 6; i++)
//     {
//         homogeneous_transform_element << cos(MDH(i, 2)), -sin(MDH(i, 2)), 0, MDH(i, 0),
//             sin(MDH(i, 2)) * cos(MDH(i, 3)), cos(MDH(i, 2)) * cos(MDH(i, 3)), -sin(MDH(i, 3)), -sin(MDH(i, 3)) * MDH(i, 1),
//             sin(MDH(i, 2)) * sin(MDH(i, 3)), cos(MDH(i, 2)) * sin(MDH(i, 3)), cos(MDH(i, 3)), cos(MDH(i, 3)) * MDH(i, 1),
//             0, 0, 0, 1;
//         for (int j = 0; j < i + 1; j++)
//         {
//             for (int k = i; k < 6; k++)
//                 homogeneous_transform(k, j) = homogeneous_transform(k, j) * homogeneous_transform_element;
//         }
//     }

//     for (int i = 0; i < 6; i++)
//     {
//         for (int j = 0; j < 6; j++)
//         {
//             homogeneous_transform_element = homogeneous_transform(i, j);
//             rotation_element << homogeneous_transform_element(0, 0), homogeneous_transform_element(0, 1), homogeneous_transform_element(0, 2),
//                 homogeneous_transform_element(1, 0), homogeneous_transform_element(1, 1), homogeneous_transform_element(1, 2),
//                 homogeneous_transform_element(2, 0), homogeneous_transform_element(2, 1), homogeneous_transform_element(2, 2);
//             translation_element << homogeneous_transform_element(0, 3), homogeneous_transform_element(1, 3), homogeneous_transform_element(2, 3);
//             rotation_joint(i, j) = rotation_element;
//             translation_joint(i, j) = translation_element;
//         }
//     }

//     for (int i = 0; i < 6; i++)
//     {
//         Jacobi_Joint2Centroid_Element = MatrixXd::Zero(6, 6);
//         Vector_Joint2Centroid_Element = Vector_Joint2Centroid(0, i);
//         Jacobi_Joint2Centroid_Element(0, i) = -Vector_Joint2Centroid_Element(1, 0);
//         Jacobi_Joint2Centroid_Element(1, i) = Vector_Joint2Centroid_Element(0, 0);
//         Jacobi_Joint2Centroid_Element(2, i) = 0;
//         Jacobi_Joint2Centroid_Element(3, i) = 0;
//         Jacobi_Joint2Centroid_Element(4, i) = 0;
//         Jacobi_Joint2Centroid_Element(5, i) = 1;

//         if (i != 0)
//         {
//             for (int j = 0; j < i; j++)
//             {
//                 rotation_element = rotation_joint(i, j + 1);
//                 translation_element = translation_joint(i, j + 1) + rotation_element * Vector_Joint2Centroid_Element;
//                 Jacobi_Joint2Centroid_Element(0, j) = -translation_element(1, 0);
//                 Jacobi_Joint2Centroid_Element(1, j) = translation_element(0, 0);
//                 Jacobi_Joint2Centroid_Element(2, j) = 0;
//                 Jacobi_Joint2Centroid_Element(3, j) = rotation_element(2, 0);
//                 Jacobi_Joint2Centroid_Element(4, j) = rotation_element(2, 1);
//                 Jacobi_Joint2Centroid_Element(5, j) = rotation_element(2, 2);
//             }
//         }

//         Jacobi_Joint2Centroid(0, i) = Jacobi_Joint2Centroid_Element;

//         homogeneous_transform_element = homogeneous_transform(5, i);
//         jacobi_joint2tool(0, i) = -homogeneous_transform_element(0, 0) * homogeneous_transform_element(1, 3) + homogeneous_transform_element(1, 0) * homogeneous_transform_element(0, 3);
//         jacobi_joint2tool(1, i) = -homogeneous_transform_element(0, 1) * homogeneous_transform_element(1, 3) + homogeneous_transform_element(1, 1) * homogeneous_transform_element(0, 3);
//         jacobi_joint2tool(2, i) = -homogeneous_transform_element(0, 2) * homogeneous_transform_element(1, 3) + homogeneous_transform_element(1, 2) * homogeneous_transform_element(0, 3);
//         jacobi_joint2tool(3, i) = homogeneous_transform_element(2, 0);
//         jacobi_joint2tool(4, i) = homogeneous_transform_element(2, 1);
//         jacobi_joint2tool(5, i) = homogeneous_transform_element(2, 2);
//     }

//     mass_matrix = MatrixXd::Zero(6, 6);

//     for (int i = 0; i < 6; i++)
//     {
//         Jacobi_Joint2Centroid_Element = Jacobi_Joint2Centroid(0, i);
//         Mass_Matrix_Centroid_Element = Mass_Matrix_Centroid(0, i);
//         Mass_Matrix_Centroid_Element = Jacobi_Joint2Centroid_Element.transpose() * Mass_Matrix_Centroid_Element * Jacobi_Joint2Centroid_Element;
//         Mass_Matrix_Joint(0, i) = Mass_Matrix_Centroid_Element;
//         mass_matrix = mass_matrix + Mass_Matrix_Centroid_Element;
//     }

//     jacobi_joint2tool = jacobi_joint2tool.inverse();
//     mass_matrix = jacobi_joint2tool.transpose() * mass_matrix * jacobi_joint2tool;

//     cout << mass_matrix << endl;
// }