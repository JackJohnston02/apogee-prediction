#ifndef CKF_CONSTANT_CB_H
#define CKF_CONSTANT_CB_H

#include <eigen3/Eigen/Dense>
#include <iostream>

#include "CKFBase.h"

class CKFConstantCB : public CKFBase
{
public:
    CKFConstantCB(const Eigen::Vector4d& initialState, const RMMatrix4x4d& initialCovariance, const double& sigma_Q,
                            const double& sigma_Q_Cb, const double& measurement_sigma_acc, const double& measurement_sigma_bar,
                            const double& t);

    std::pair<Eigen::Vector4d, RMMatrix4x4d> updateAccelerometer(const double measurement, const double t_current) override;
private:
    RMMatrix4x8d processModel(const RMMatrix4x8d& cubaturePoints, const double dt) override;
};

#endif // CKF_CONSTANT_CB_H