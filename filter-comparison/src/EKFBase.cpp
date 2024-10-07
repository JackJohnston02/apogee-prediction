#include "EKFBase.h"

EKFBase::EKFBase(const Eigen::Vector4d& initialState, const RMMatrix4x4d& initialCovariance, const double& sigma_Q,
                                                 const double& sigma_Q_Cb, const double& measurement_sigma_acc, const double& measurement_sigma_bar,
                                                 const double& t)
    : KalmanFilter(initialState, initialCovariance, sigma_Q, sigma_Q_Cb, measurement_sigma_acc, measurement_sigma_bar, t)
{}

void EKFBase::predict(const double t_current)
{
    const double dt = t_current - this->t_last_update;
    this->t_last_update = t_current;

    this->Q = calculateProcessNoise(dt);

    Eigen::Vector4d predictedState;
    RMMatrix4x4d predictedCovariance;

    std::tie(predictedState, predictedCovariance) = this->processModel(this->x, this->P, dt);

    this->x = predictedState;
    this->P = predictedCovariance + this->Q;
}

std::pair<Eigen::Vector4d, RMMatrix4x4d> EKFBase::updateAccelerometer(const double measurement, const double t_current) 
{
    const double adjustedMeasurement = measurement + this->getGravity(this->x(0));

    const Eigen::RowVector4d H(0, 0, 1, 0);

    Eigen::Vector4d predictedState;
    RMMatrix4x4d predictedCovariance;
    std::tie(predictedState, predictedCovariance) = this->processModel(this->x, this->P, t_current - this->t_last_update);
    
    const double S = H * predictedCovariance * H.transpose() + this->R_acc;
    const Eigen::Vector4d K = predictedCovariance * H.transpose() / S;

    const double innovation = adjustedMeasurement - H * predictedState;
    const Eigen::Vector4d updatedState = predictedState + K * innovation;
    const RMMatrix4x4d updatedCovariance = predictedCovariance - K * S * K.transpose();
    this->x = updatedState;
    this->P = updatedCovariance;

    this->t_last_update = t_current;

    return {updatedState, updatedCovariance};
}

std::pair<Eigen::Vector4d, RMMatrix4x4d> EKFBase::updateBarometer(const double measurement, const double t_current)
{
    const Eigen::RowVector4d H(1, 0, 0, 0);

    Eigen::Vector4d predictedState;
    RMMatrix4x4d predictedCovariance;
    std::tie(predictedState, predictedCovariance) = this->processModel(this->x, this->P, t_current - this->t_last_update);

    const double S = H * predictedCovariance * H.transpose() + this->R_bar;
    const Eigen::Vector4d K = predictedCovariance * H.transpose() / S;

    const double innovation = measurement - H * predictedState;
    const Eigen::Vector4d updatedState = predictedState + K * innovation;
    const RMMatrix4x4d updatedCovariance = predictedCovariance - K * S * K.transpose();
    this->x = updatedState;
    this->P = updatedCovariance;

    this->t_last_update = t_current;

    return {updatedState, updatedCovariance};
}

std::pair<double, double> EKFBase::getApogee()
{
    Eigen::Vector4d propagatedX = this->x;
    RMMatrix4x4d propagatedP = this->P;
    RMMatrix4x4d Q_apa = this->calculateProcessNoise(this->dt_apa);
    while (propagatedX(1) > 0 && propagatedX(2) < g_0)
    {
        std::tie(propagatedX, propagatedP) = this->processModel(propagatedX, propagatedP, this->dt_apa);
        propagatedP += Q_apa;
    }

    return {propagatedX(0), std::sqrt(propagatedP(0,0))};
}