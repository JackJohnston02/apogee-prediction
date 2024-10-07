#ifndef ROW_MAJOR_MATRIX_H
#define ROW_MAJOR_MATRIX_H

#include <eigen3/Eigen/Dense>

// Row Major definitions of 4x4 and 4x8 matrices 
// As Eigen using column major ordering by default
// Used to mimic matlab code as close as possible
typedef Eigen::Matrix<double, 4, 4, Eigen::RowMajor> RMMatrix4x4d;
typedef Eigen::Matrix<double, 4, 9, Eigen::RowMajor> RMMatrix4x9d;
typedef Eigen::Matrix<double, 4, 8, Eigen::RowMajor> RMMatrix4x8d;
typedef Eigen::Matrix<double, 9, 1> RMVector9x1d; // technically vectors are row major anyway? just naming it RM... for consistency 
typedef Eigen::RowVector<double, 9> RMVector1x9d;
typedef Eigen::RowVector<double, 8> RMVector1x8d;

#endif