#ifndef DYNAMIC_COMPUTATION_H
#define DYNAMIC_COMPUTATION_H

#include <cmath>
#include "std_msgs/String.h"
#include "Eigen/Core"
#include "Eigen/Geometry"

using namespace std;
using namespace Eigen;

MatrixXd MassMatrixComputation(MatrixXd, MatrixXd);
MatrixXd JacobianMatrixComputation(MatrixXd);

#endif