#ifndef MDK_COMPUTATION_H
#define MDK_COMPUTATION_H

#include <cmath>
#include "std_msgs/String.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/unsupported/MatrixFunctions"

using namespace std;
using namespace Eigen;

class MDK
{
public:
    Matrix<double, 6, 6> M_;
    Matrix<double, 6, 6> D_;
    Matrix<double, 6, 6> K_;
};

MDK MDKComputation(MatrixXd, MatrixXd);

#endif