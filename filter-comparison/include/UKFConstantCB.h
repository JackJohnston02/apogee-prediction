#ifndef UKF_CONSTANT_CB_H
#define UKF_CONSTANT_CB_H

#include <eigen3/Eigen/Dense>
#include <iostream>

#include "UKFBase.h"
#include "RowMajorMatrix.h"

class UKFConstantCB : public UKFBase
{
public:
    UKFConstantCB(const Eigen::Vector4d& initialState, const RMMatrix4x4d& initialCovariance, const double& sigma_Q,
                            const double& sigma_Q_Cb, const double& measurement_sigma_acc, const double& measurement_sigma_bar,
                            const double& t);
    
private:
    RMMatrix4x9d processModel(const RMMatrix4x9d& sigmaPoints, const double& dt) override;
};

#endif // UKF_CONSTANT_CB_H