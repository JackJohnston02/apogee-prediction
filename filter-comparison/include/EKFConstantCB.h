#ifndef EKF_CONSTANT_CB_H
#define EKF_CONSTANT_CB_H

#include <eigen3/Eigen/Dense>

#include "EKFBase.h"
#include "RowMajorMatrix.h"

class EKFConstantCB : public EKFBase 
{
public:
    EKFConstantCB(const Eigen::Vector4d& initialState, const RMMatrix4x4d& initialCovariance, const double& sigma_Q,
                  const double& sigma_Q_Cb, const double& measurement_sigma_acc, const double& measurement_sigma_bar,
                  const double& t);
private:    
    std::pair<Eigen::Vector4d, RMMatrix4x4d> processModel(const Eigen::Vector4d& x, const RMMatrix4x4d& P, const double& dt) override;
};

#endif // EKF_CONSTANT_CB_H