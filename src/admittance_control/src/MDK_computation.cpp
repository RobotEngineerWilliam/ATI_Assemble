#include "MDK_computation.h"

using namespace std;
using namespace Eigen;

const double PI = 3.1415926;

#pragma region /*Parameter for Mass Matrix Computation*/

#pragma endregion

MDK MDKComputation(MatrixXd mass_matrix, MatrixXd K_reference)
{
    MDK MDK_output;

    MDK_output.M_ = mass_matrix;
    MDK_output.D_ = mass_matrix * K_reference;
    MDK_output.D_ = 2 * MDK_output.D_.sqrt();
    MDK_output.K_ = K_reference;

    return MDK_output;
}