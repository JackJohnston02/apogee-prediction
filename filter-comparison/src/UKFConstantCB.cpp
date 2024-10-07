#include "UKFConstantCB.h"

UKFConstantCB::UKFConstantCB(const Eigen::Vector4d& initialState, const RMMatrix4x4d& initialCovariance, const double& sigma_Q,
                                                 const double& sigma_Q_Cb, const double& measurement_sigma_acc, const double& measurement_sigma_bar,
                                                 const double& t)
    : UKFBase(initialState, initialCovariance, sigma_Q, sigma_Q_Cb, measurement_sigma_acc, measurement_sigma_bar, t) 
{}

RMMatrix4x9d UKFConstantCB::processModel(const RMMatrix4x9d& sigmaPoints, const double& dt)
{
    RMMatrix4x9d newSigmaPoints;

    for (int i = 0; i < sigmaPoints.cols(); i++)
    {
        Eigen::Vector4d x_s = sigmaPoints.col(i);

        const double g = this->getGravity(x_s(0));
        const double rho = this->getDensity(x_s(0));

        x_s(0) = x_s(0) + x_s(1) * dt + 0.5 * x_s(2) * std::pow(dt,2);
        x_s(1) = x_s(1) + x_s(2) * dt;

        if (x_s(2) < g)
        {
            x_s(2) = g - (rho * std::pow(x_s(1),2)) / (2 * x_s(3));
            x_s(3) = std::max(x_s(3), 10.0);
        }

        newSigmaPoints.col(i) = x_s;
    }

    return newSigmaPoints;
}
    
