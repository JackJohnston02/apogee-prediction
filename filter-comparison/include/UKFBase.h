#ifndef UKF_BASE_H
#define UKF_BASE_H

#include <eigen3/Eigen/Dense>
#include <iostream>

#include "KalmanFilter.h"
#include "RowMajorMatrix.h"

class UKFBase : public KalmanFilter
{
public:
    UKFBase(const Eigen::Vector4d& initialState, const RMMatrix4x4d& initialCovariance, const double& sigma_Q,
                            const double& sigma_Q_Cb, const double& measurement_sigma_acc, const double& measurement_sigma_bar,
                            const double& t);

    void predict(const double t_current) override;
    std::pair<double, double> getApogee() override;
    std::pair<Eigen::Vector4d, RMMatrix4x4d> updateAccelerometer(const double measurement, const double t_current) override;    
    std::pair<Eigen::Vector4d, RMMatrix4x4d> updateBarometer(const double measurement, const double t_current) override;

protected:
    const double alpha = 1e-3;
    const double beta = 2;
    const double kappa = 0;

    virtual RMMatrix4x9d processModel(const RMMatrix4x9d& sigmaPoints, const double& dt) = 0;
    
    RMVector1x9d measurementModelBarometer(const RMMatrix4x9d& sigmaPoints);
    RMVector1x9d measurementModelAccelerometer(const RMMatrix4x9d& sigmaPoints);
    std::tuple<RMMatrix4x9d, RMVector9x1d, RMVector9x1d> generateSigmaPoints(const Eigen::Vector4d& mean, const RMMatrix4x4d& covariance); 
    RMMatrix4x4d calculateCovariance(const RMMatrix4x9d& sigmaPoints, const Eigen::Vector4d& mean, const RMVector9x1d& weightsCovariance, const RMMatrix4x4d& noiseCovariance);
    double calculateCovariance(const RMVector1x9d& sigmaPoints, const double mean, const RMVector9x1d& weightsCovariance, const double noiseCovariance);
    Eigen::Vector4d calculateCrossCovariance(const RMMatrix4x9d& sigmaPoints, const Eigen::Vector4d& stateMean, const RMVector1x9d& predictedMeasurementSigmaPoints, const double predictedMeasurement, const RMVector9x1d& weightsCovariance);

};

#endif // UKF_BASE_H