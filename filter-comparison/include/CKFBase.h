#ifndef CKF_BASE_H
#define CKF_BASE_H

#include <iostream>

#include "KalmanFilter.h"

class CKFBase : public KalmanFilter
{
public:
    CKFBase(const Eigen::Vector4d& initialState, const RMMatrix4x4d& initialCovariance, const double& sigma_Q,
            const double& sigma_Q_Cb, const double& measurement_sigma_acc, const double& measurement_sigma_bar,
            const double& t);

    void predict(const double t_current) override;
    std::pair<double, double> getApogee() override;
    std::pair<Eigen::Vector4d, RMMatrix4x4d> updateAccelerometer(const double measurement, const double t_current) override;
    std::pair<Eigen::Vector4d, RMMatrix4x4d> updateBarometer(const double measurement, const double t_current) override;

protected:
    virtual RMMatrix4x8d processModel(const RMMatrix4x8d& cubaturePoints, const double dt) = 0;

    RMVector1x8d measurementModelAccelerometer(const RMMatrix4x8d& cubaturePoints); 
    RMVector1x8d measurementModelBarometer(const RMMatrix4x8d& cubaturePoints);
    RMMatrix4x8d generateCubaturePoints(const Eigen::Vector4d& mean, const RMMatrix4x4d& covariance);
    RMMatrix4x4d calculateCovariance(const RMMatrix4x8d& points, const Eigen::Vector4d& mean, const RMMatrix4x4d& noiseCovariance);
    double calculateCovariance(const RMVector1x8d& points, const double mean, const double noise_covariance);
    Eigen::Vector4d calculateCrossCovariance(const RMMatrix4x8d& statePoints, const Eigen::Vector4d& stateMean, const RMVector1x8d& measurementPoints, const double measurementMean);
};  

#endif