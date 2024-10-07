#ifndef EKF_BASE_H
#define EKF_BASE_H

#include <eigen3/Eigen/Dense>

#include "KalmanFilter.h"
#include "RowMajorMatrix.h"

class EKFBase : public KalmanFilter
{

public:
    EKFBase(const Eigen::Vector4d& initialState, const RMMatrix4x4d& initialCovariance, const double& sigma_Q,
            const double& sigma_Q_Cb, const double& measurement_sigma_acc, const double& measurement_sigma_bar,
            const double& t);

    void predict(const double t_current) override;
    std::pair<double, double> getApogee() override;
    std::pair<Eigen::Vector4d, RMMatrix4x4d> updateAccelerometer(const double measurement, const double t_current) override;
    std::pair<Eigen::Vector4d, RMMatrix4x4d> updateBarometer(const double measurement, const double t_current) override;

protected:
    virtual std::pair<Eigen::Vector4d, RMMatrix4x4d> processModel(const Eigen::Vector4d& x, const RMMatrix4x4d& P, const double& dt) = 0;
};

#endif // EKF_BASE_H