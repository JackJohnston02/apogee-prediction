#include "EKFConstantCB.h"

EKFConstantCB::EKFConstantCB(const Eigen::Vector4d& initialState, const RMMatrix4x4d& initialCovariance, const double& sigma_Q,
                             const double& sigma_Q_Cb, const double& measurement_sigma_acc, const double& measurement_sigma_bar,
                             const double& t)
    : EKFBase(initialState, initialCovariance, sigma_Q, sigma_Q_Cb, measurement_sigma_acc, measurement_sigma_bar, t)
{}

std::pair<Eigen::Vector4d, RMMatrix4x4d> EKFConstantCB::processModel(const Eigen::Vector4d& x, const RMMatrix4x4d& P, const double& dt)
{
    const double rho = this->getDensity(x(0));
    const double g = this->getGravity(x(0));
    const Eigen::Vector4d B(0, 0, 0, 1);
    
    double u = 0;
    RMMatrix4x4d F;
    if (x(2) < g)
    {
        F <<         1,                    dt, 0.5*std::pow(dt,2),                                   0,
                     0,                     1,                 dt,                                   0,
                     0, - (rho * x(1))/(x(3)),                  0, (rho*x(1)*x(1)) / (2 * x(3) * x(3)),
                     0,                     0,                  0,                                   1;

        u = this->getGravity(x(0));
    }
    else
    {
        F << 1, dt, 0.5*std::pow(dt,2), 0,
             0,  1,                 dt, 0,
             0,  0,                  1, 0,
             0,  0,                  0, 1;
    }

    const Eigen::Vector4d newX = F * x + B * u;
    const RMMatrix4x4d newP = F * P * F.transpose() + this->Q;

    return {newX, newP};
}