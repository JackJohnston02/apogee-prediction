#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(const Eigen::Vector4d& initialState, const RMMatrix4x4d& initialCovariance, const double& sigma_Q,
             const double& sigma_Q_Cb, const double& measurement_sigma_acc, const double& measurement_sigma_bar,
             const double& t)
    : x(initialState),
      P(initialCovariance),
      t_last_update(t),
      sigma_Q(sigma_Q),
      sigma_Q_Cb(sigma_Q_Cb)
{
    RMMatrix4x4d diagonalMatrix = RMMatrix4x4d::Zero();
    diagonalMatrix(0, 0) = 1e-3;
    diagonalMatrix(1, 1) = 1e-3;
    diagonalMatrix(2, 2) = 1e-2;
    diagonalMatrix(3, 3) = 1e1;

    this->Q = sigma_Q * diagonalMatrix;
    this->R_acc = measurement_sigma_acc * measurement_sigma_acc;
    this->R_bar = measurement_sigma_bar * measurement_sigma_bar;
}

double KalmanFilter::getDensity(const double height)
{
    const double g = this->getGravity(height);

    return (p_0 * M) / (R * T_0) * std::pow((1 - (L * height) / T_0), (((-g * M) / (R * L)) - 1));
}

double KalmanFilter::getGravity(const double height)
{
    return g_0 * std::pow((R_e / (R_e + height)), 2);
}

RMMatrix4x4d KalmanFilter::calculateProcessNoise(const double& dt)
{
    Eigen::Matrix4d Q;

    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt3 * dt;
    const double variance_Q = this->sigma_Q * this->sigma_Q;
    
    const double g = this->getGravity(this->x(0)); 
    double Q_Cb = 0;

    if (g - this->x(2) > 0)
    {
        Q_Cb = std::pow(this->sigma_Q_Cb, 2);
    }

    Q << 1.0 / 4.0 * dt4 * variance_Q, 1.0 / 2.0 * dt3 * variance_Q, 1.0 / 2.0 * dt2 * variance_Q, 0,
         1.0 / 2.0 * dt3 * variance_Q, dt2 * variance_Q,             dt * variance_Q,              0,
         1.0 / 2.0 * dt2 * variance_Q, dt * variance_Q,              variance_Q,                   0,
         0,                            0,                            0,                            Q_Cb;

    return Q;
}