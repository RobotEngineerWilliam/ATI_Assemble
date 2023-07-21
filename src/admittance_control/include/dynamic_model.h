#ifndef DYNAMIC_MODEL_H
#define DYNAMIC_MODEL_H

#include <cmath>
#include "std_msgs/String.h"
#include "Eigen/Core"
#include "Eigen/Geometry"

using namespace std;
using namespace Eigen;

MatrixXd MassMatrixComputation(MatrixXd, MatrixXd);

#endif