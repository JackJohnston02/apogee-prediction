#include "CKFConstantCB.h"

CKFConstantCB::CKFConstantCB(const Eigen::Vector4d& initialState, const RMMatrix4x4d& initialCovariance, const double& sigma_Q,
                             const double& sigma_Q_Cb, const double& measurement_sigma_acc, const double& measurement_sigma_bar,
                             const double& t)
    : CKFBase(initialState, initialCovariance, sigma_Q, sigma_Q_Cb, measurement_sigma_acc, measurement_sigma_bar, t) 
{}

std::pair<Eigen::Vector4d, RMMatrix4x4d> CKFConstantCB::updateAccelerometer(const double measurement, const double t_current)
{
    const double adjustedMeasurement = measurement + this->getGravity(this->x(0));
    this->t_last_update = t_current;

    const RMMatrix4x8d cubaturePoints = this->generateCubaturePoints(this->x, this->P);

    const RMVector1x8d predictedMeasurementPoints = this->measurementModelAccelerometer(cubaturePoints);
    const double predictedMeasurement = predictedMeasurementPoints.rowwise().mean().value();

    const double innovationCovariance = this->calculateCovariance(predictedMeasurementPoints, predictedMeasurement, this->R_acc);
    const Eigen::Vector4d crossCovariance = this->calculateCrossCovariance(cubaturePoints, this->x, predictedMeasurementPoints, predictedMeasurement);
    
    const Eigen::Vector4d kalmanGain = crossCovariance / innovationCovariance;
    const double innovation = adjustedMeasurement - predictedMeasurement;

    const Eigen::Vector4d updatedState = this->x + (kalmanGain * innovation);
    const Eigen::Matrix4d updatedCovariance = this->P - (kalmanGain * innovationCovariance * kalmanGain.transpose());

    this->x = updatedState;
    this->P = updatedCovariance;

    return {updatedState, updatedCovariance};
}

RMMatrix4x8d CKFConstantCB::processModel(const RMMatrix4x8d& cubaturePoints, const double dt)
{
    RMMatrix4x8d predictedCubaturePoints;

    const int numCubaturePoints = cubaturePoints.cols();

    for (int i = 0; i < numCubaturePoints; i++)
    {
        Eigen::Vector4d x_s = cubaturePoints.col(i);

        const double g = this->getGravity(x_s(0));
        const double rho = this->getDensity(x_s(0));

        x_s(0) = x_s(0) + x_s(1) * dt + 0.5 * x_s(2) * std::pow(dt, 2);
        x_s(1) = x_s(1) + x_s(2) * dt;

        if (x_s(2) < g)
        {
            x_s(2) = g - (rho * std::pow(x_s(1), 2)) / (2 * x_s(3)); 
            x_s(3) = std::max(x_s(3), 10.0);
        }
        
        predictedCubaturePoints.col(i) = x_s;
    }

    return predictedCubaturePoints;
}