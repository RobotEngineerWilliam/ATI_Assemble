#ifndef JAKA_CONVERSION_H
#define JAKA_CONVERSION_H

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/StdVector"
#include "cmath"
#include "iostream"
#include "robot.h"

RotMatrix Angaxis2Rot(Eigen::Vector3d &agax3);

Eigen::Vector3d Rot2Angaxis(RotMatrix &r);

#endif

