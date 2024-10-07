#include "CKFBase.h"

CKFBase::CKFBase(const Eigen::Vector4d& initialState, const RMMatrix4x4d& initialCovariance, const double& sigma_Q,
                                                 const double& sigma_Q_Cb, const double& measurement_sigma_acc, const double& measurement_sigma_bar,
                                                 const double& t)
    : KalmanFilter(initialState, initialCovariance, sigma_Q, sigma_Q_Cb, measurement_sigma_acc, measurement_sigma_bar, t) 
{}

void CKFBase::predict(const double t_current)
{
    const double dt = t_current - this->t_last_update;
    this->t_last_update = t_current;
    
    const RMMatrix4x8d cubaturePoints = this->generateCubaturePoints(this->x, this->P);
    
    const RMMatrix4x8d predictedCubaturePoints = this->processModel(cubaturePoints, dt);

    const Eigen::Vector4d predictedState = predictedCubaturePoints.rowwise().mean();

    this->Q = calculateProcessNoise(dt);
    const Eigen::Matrix4d predictedCovariance = this->calculateCovariance(predictedCubaturePoints, predictedState, this->Q);
    
    this->x = predictedState;
    this->P = predictedCovariance;
}

std::pair<double, double> CKFBase::getApogee()
{
    Eigen::Vector4d propagated_x = this->x;
    RMMatrix4x4d propagated_P = this->P;

    while (propagated_x(1) > 0 && propagated_x(2) < g_0)
    {
        const RMMatrix4x8d cubaturePoints = this->generateCubaturePoints(propagated_x, propagated_P);
        
        const RMMatrix4x8d predictedCubaturePoints = this->processModel(cubaturePoints, this->dt_apa);

        const Eigen::Vector4d predictedState = predictedCubaturePoints.rowwise().mean();

        const RMMatrix4x4d predictedCovariance = this->calculateCovariance(predictedCubaturePoints, predictedState, this->Q);

        propagated_x = predictedState;
        propagated_P = predictedCovariance;
    }

    const double apogee = propagated_x(0);
    const double apogeeStd = std::sqrt(propagated_P(0, 0));

    return {apogee, apogeeStd};
}

RMVector1x8d CKFBase::measurementModelAccelerometer(const RMMatrix4x8d& cubaturePoints)
{
    return cubaturePoints.row(2);
}

RMVector1x8d CKFBase::measurementModelBarometer(const RMMatrix4x8d& cubaturePoints)
{
    return cubaturePoints.row(0);
}

RMMatrix4x8d CKFBase::generateCubaturePoints(const Eigen::Vector4d& mean, const RMMatrix4x4d& covariance)
{
    const int n = mean.rows();
    RMMatrix4x8d cubaturePoints = RMMatrix4x8d::Zero();

    Eigen::LDLT<Eigen::Matrix4d> covChol(covariance); 
    if (covChol.info() == Eigen::Success)
    {
        Eigen::MatrixXd lowerTriangular = covChol.matrixL();

        for (int i = 0; i < n; i++)
        {
            cubaturePoints.col(i) = mean + std::sqrt(n) * lowerTriangular.col(i);
            cubaturePoints.col(i + n) = mean - std::sqrt(n) * lowerTriangular.col(i);
        }
    }
    else
    {
        std::cerr << "Cholesky Decomposition Failed!" << std::endl; 
    }

    return cubaturePoints;
}

std::pair<Eigen::Vector4d, RMMatrix4x4d> CKFBase::updateBarometer(const double measurement, const double t_current)
{
    this->t_last_update = t_current;
    
    const RMMatrix4x8d cubaturePoints = this->generateCubaturePoints(this->x, this->P);
    const RMVector1x8d predictedMeasurementPoints = this->measurementModelBarometer(cubaturePoints);
    const double predictedMeasurement = predictedMeasurementPoints.mean();

    const double innovationCovariance = this->calculateCovariance(predictedMeasurementPoints, predictedMeasurement, this->R_bar);

    const Eigen::Vector4d crossCovariance = this->calculateCrossCovariance(cubaturePoints, this->x, predictedMeasurementPoints, predictedMeasurement);

    const Eigen::Vector4d kalmanGain = crossCovariance / innovationCovariance;

    const double innovation = measurement - predictedMeasurement;

    const Eigen::Vector4d updatedState = this->x + kalmanGain * innovation;
    const RMMatrix4x4d updatedCovariance = this->P - kalmanGain * innovationCovariance * kalmanGain.transpose();

    this->x = updatedState;
    this->P = updatedCovariance;

    return {updatedState, updatedCovariance};
}

RMMatrix4x4d CKFBase::calculateCovariance(const RMMatrix4x8d& points, const Eigen::Vector4d& mean, const RMMatrix4x4d& noiseCovariance)
{
    const int n = points.cols();  

    RMMatrix4x4d covariance = RMMatrix4x4d::Zero();

    for (int i = 0; i < n; i++)
    {
        Eigen::Vector4d diff = points.col(i) - mean;
        covariance += diff * diff.transpose();
    }

    covariance /= n;
    covariance += noiseCovariance;

    return covariance;
}

double CKFBase::calculateCovariance(const RMVector1x8d& points, const double mean, const double noise_covariance)
{
    const int n = points.size();  
    double covariance = 0.0;

    for (int i = 0; i < n; i++)
    {
        const double diff = points(i) - mean;
        covariance += diff * diff;
    }

    covariance /= n;
    covariance += noise_covariance;

    return covariance;
}

Eigen::Vector4d CKFBase::calculateCrossCovariance(const RMMatrix4x8d& statePoints, const Eigen::Vector4d& stateMean, const RMVector1x8d& measurementPoints, const double measurementMean)
{
    const int n = statePoints.cols();
    
    Eigen::Vector4d crossCovariance = Eigen::Vector4d::Zero();

    for (int i = 0; i < n; i++)
    {
        const Eigen::Vector4d stateDiff = statePoints.col(i) - stateMean;
        const double measurementDiff = measurementPoints(i) - measurementMean;
        crossCovariance += stateDiff * measurementDiff;
    }

    crossCovariance /= n;

    return crossCovariance;
}

std::pair<Eigen::Vector4d, RMMatrix4x4d> CKFBase::updateAccelerometer(const double measurement, const double t_current)
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