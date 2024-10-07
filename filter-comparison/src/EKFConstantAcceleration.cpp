#include "EKFConstantAcceleration.h"

EKFConstantAcceleration::EKFConstantAcceleration(const Eigen::Vector4d& initialState, const RMMatrix4x4d& initialCovariance, const double& sigma_Q,
                                                 const double& sigma_Q_Cb, const double& measurement_sigma_acc, const double& measurement_sigma_bar,
                                                 const double& t)
    : EKFBase(initialState, initialCovariance, sigma_Q, sigma_Q_Cb, measurement_sigma_acc, measurement_sigma_bar, t)
{}

std::pair<Eigen::Vector4d, RMMatrix4x4d> EKFConstantAcceleration::processModel(const Eigen::Vector4d& x, const RMMatrix4x4d& P, const double& dt)
{
    const double rho = this->getDensity(x(0));
    const double g = this->getGravity(x(0));

    const Eigen::Vector4d B(0, 0, 0, 1);

    const double u = (rho * x(1) * x(1)) / (2 * -(x(2) - g));

    RMMatrix4x4d F;

    F << 1, dt, 0.5*dt * 0.5*dt, 0,
         0,  1,              dt, 0,
         0,  0,               1, 0,
         0,  0,               0, 0;

    const Eigen::Vector4d new_X = F * x + B * u;
    const RMMatrix4x4d new_P = F * P * F.transpose();

    return {new_X, new_P};
}