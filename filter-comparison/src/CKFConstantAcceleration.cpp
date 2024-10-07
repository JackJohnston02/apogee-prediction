#include "CKFConstantAcceleration.h"

CKFConstantAcceleration::CKFConstantAcceleration(const Eigen::Vector4d& initialState, const RMMatrix4x4d& initialCovariance, const double& sigma_Q,
                                                 const double& sigma_Q_Cb, const double& measurement_sigma_acc, const double& measurement_sigma_bar,
                                                 const double& t)
    : CKFBase(initialState, initialCovariance, sigma_Q, sigma_Q_Cb, measurement_sigma_acc, measurement_sigma_bar, t) 
{}

RMMatrix4x8d CKFConstantAcceleration::processModel(const RMMatrix4x8d& cubaturePoints, const double dt)
{
    RMMatrix4x8d predictedCubaturePoints;

    const int numCubaturePoints = cubaturePoints.cols();

    for (int i = 0; i < numCubaturePoints; i++)
    {
        Eigen::Vector4d x_s = cubaturePoints.col(i);

        x_s[0] = x_s[0] + x_s[1] * dt + 0.5 * x_s[2] * dt * dt;
        x_s[1] = x_s[1] + x_s[2] * dt;
        
        predictedCubaturePoints.col(i) = x_s;
    }

    return predictedCubaturePoints;
}