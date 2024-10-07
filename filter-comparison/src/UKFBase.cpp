#include "UKFBase.h"

UKFBase::UKFBase(const Eigen::Vector4d& initialState, const RMMatrix4x4d& initialCovariance, const double& sigma_Q,
                                                 const double& sigma_Q_Cb, const double& measurement_sigma_acc, const double& measurement_sigma_bar,
                                                 const double& t)
    : KalmanFilter(initialState, initialCovariance, sigma_Q, sigma_Q_Cb, measurement_sigma_acc, measurement_sigma_bar, t) 
{}

void UKFBase::predict(const double t_current)
{
    const double dt = t_current - this->t_last_update;
    this->t_last_update = t_current;

    RMMatrix4x9d sigmaPoints;
    RMVector9x1d weightsMean;
    RMVector9x1d weightsCovariance;

    std::tie(sigmaPoints, weightsMean, weightsCovariance) = this->generateSigmaPoints(this->x, this->P);

    const RMMatrix4x9d predictedSigmaPoints = this->processModel(sigmaPoints, dt);

    const Eigen::Vector4d predictedState = predictedSigmaPoints * weightsMean;

    this->Q = this->calculateProcessNoise(dt);
    const RMMatrix4x4d predictedCovariance = this->calculateCovariance(predictedSigmaPoints, predictedState, weightsCovariance, this->Q);

    this->x = predictedState;
    this->P = predictedCovariance;
}

std::pair<double, double> UKFBase::getApogee()
{
    Eigen::Vector4d propagatedX = this->x;
    RMMatrix4x4d propagatedP = this->P;

    RMMatrix4x9d sigmaPoints;
    RMVector9x1d weightsMean;
    RMVector9x1d weightsCovariance;

    while (propagatedX(1) > 0 && propagatedX(2) < g_0)
    {
        std::tie(sigmaPoints, weightsMean, weightsCovariance) = this->generateSigmaPoints(propagatedX, propagatedP);

        RMMatrix4x9d predictedSigmaPoints = this->processModel(sigmaPoints, this->dt_apa);

        Eigen::Vector4d predictedState = predictedSigmaPoints * weightsMean;

        RMMatrix4x4d predictedCovariance = this->calculateCovariance(predictedSigmaPoints, predictedState, weightsCovariance, this->Q);

        propagatedX = predictedState;
        propagatedP = predictedCovariance;
    }

    return {propagatedX(0), std::sqrt(propagatedP(0,0))};
}

RMVector1x9d UKFBase::measurementModelAccelerometer(const RMMatrix4x9d& sigmaPoints)
{
    RMVector1x9d predictedMeasurementSigmaPoints = RMVector1x9d::Zero(sigmaPoints.cols());

    for (int i = 0; i < sigmaPoints.cols(); i++)
    {
        const Eigen::Vector4d x = sigmaPoints.col(i);
        const double z = x(2);
        predictedMeasurementSigmaPoints(i) = z;
    }

    return predictedMeasurementSigmaPoints;
}

std::pair<Eigen::Vector4d, RMMatrix4x4d> UKFBase::updateBarometer(const double measurement, const double t_current)
{
    this->t_last_update = t_current;

    RMMatrix4x9d sigmaPoints;
    RMVector9x1d weightsMean;
    RMVector9x1d weightsCovariance;
    std::tie(sigmaPoints, weightsMean, weightsCovariance) = this->generateSigmaPoints(this->x, this->P);

    const RMVector1x9d predictedMeasurementSigmaPoints = this->measurementModelBarometer(sigmaPoints);
    const double predictedMeasurement = predictedMeasurementSigmaPoints * weightsMean;
    
    const double innovationCovariance = this->calculateCovariance(predictedMeasurementSigmaPoints, predictedMeasurement, weightsCovariance, this->R_bar);
    const Eigen::Vector4d crossCovariance = this->calculateCrossCovariance(sigmaPoints, this->x, predictedMeasurementSigmaPoints, predictedMeasurement, weightsCovariance);
    const Eigen::Vector4d kalmanGain = crossCovariance / innovationCovariance;
    const double innovation = measurement - predictedMeasurement;

    const Eigen::Vector4d updatedState = this->x + kalmanGain * innovation;
    const RMMatrix4x4d updatedCovariance = this->P - kalmanGain * innovationCovariance * kalmanGain.transpose();

    this->x = updatedState;
    this->P = updatedCovariance;

    return {updatedState, updatedCovariance};
}

RMVector1x9d UKFBase::measurementModelBarometer(const RMMatrix4x9d& sigmaPoints)
{
    RMVector1x9d predictedMeasurementSigmaPoints = RMVector1x9d::Zero(sigmaPoints.cols());

    for (int i = 0; i < sigmaPoints.cols(); i++)
    {
        const Eigen::Vector4d x = sigmaPoints.col(i);
        const double z = x(0);
        predictedMeasurementSigmaPoints(i) = z;
    }

    return predictedMeasurementSigmaPoints;
}

std::tuple<RMMatrix4x9d, RMVector9x1d, RMVector9x1d> UKFBase::generateSigmaPoints(const Eigen::Vector4d& mean, const RMMatrix4x4d& covariance)
{
    const int n = mean.size();
    const double lambda = std::pow(this->alpha,2) * (n + this->kappa) - n;
    RMMatrix4x9d sigmaPoints = RMMatrix4x9d::Zero(n, 2*n + 1);
    RMVector9x1d weightsMean = RMVector9x1d::Zero(2*n + 1, 1);
    RMVector9x1d weightsCovariance = RMVector9x1d::Zero(2*n + 1, 1);

    sigmaPoints.col(0) = mean;
    weightsMean(0) = lambda / (n + lambda);
    weightsCovariance(0) = weightsMean(0) + (1 - std::pow(this->alpha,2) + this->beta);

    Eigen::LDLT<Eigen::Matrix4d> covChol(covariance);
    if (covChol.info() == Eigen::Success)
    {
        const RMMatrix4x4d lowerTriangular = covChol.matrixL();

        for (int i = 0; i < n; i++)
        {
            sigmaPoints.col(i+1) = mean + lowerTriangular.col(i); 
            sigmaPoints.col(i+n+1) = mean - lowerTriangular.col(i);
            weightsMean(i+1) = 1 / (2 * (n + lambda));
            weightsCovariance(i+1) = weightsMean(i+1);
            weightsMean(i+n+1) = 1 / (2 * (n + lambda));
            weightsCovariance(i+n+1) = weightsMean(i+1);
        }
    }
    else
    {
        std::cerr << "Cholesky Decomposition Failed!" << std::endl; 
    }

    return {sigmaPoints, weightsMean, weightsCovariance};
} 

RMMatrix4x4d UKFBase::calculateCovariance(const RMMatrix4x9d& sigmaPoints, const Eigen::Vector4d& mean, const RMVector9x1d& weightsCovariance, const RMMatrix4x4d& noiseCovariance)
{
    const int n = sigmaPoints.rows(); 
    RMMatrix4x4d covariance = RMMatrix4x4d::Zero(n, n); 

    for (int i = 0; i < sigmaPoints.cols(); ++i)
    {
        Eigen::Vector4d diff = sigmaPoints.col(i) - mean; 
        covariance += weightsCovariance(i) * (diff * diff.transpose()); 
    }

    covariance += noiseCovariance;

    return covariance;
}

double UKFBase::calculateCovariance(const RMVector1x9d& sigmaPoints, const double mean, const RMVector9x1d& weightsCovariance, const double noiseCovariance)
{
    double covariance = 0.0;

    for (int i = 0; i < sigmaPoints.size(); ++i)
    {
        double diff = sigmaPoints(i) - mean; 
        covariance += weightsCovariance(i) * (diff * diff); 
    }

    covariance += noiseCovariance;

    return covariance;
}

Eigen::Vector4d UKFBase::calculateCrossCovariance(const RMMatrix4x9d& sigmaPoints, const Eigen::Vector4d& stateMean, const RMVector1x9d& predictedMeasurementSigmaPoints, const double predictedMeasurement, const RMVector9x1d& weightsCovariance)
{
    Eigen::Vector4d crossCovariance = Eigen::Vector4d::Zero();

    for (int i = 0; i < sigmaPoints.cols(); i++)
    {
        Eigen::Vector4d stateDiff = sigmaPoints.col(i) - stateMean;
        const double measurementDiff = predictedMeasurementSigmaPoints.col(i).value() - predictedMeasurement;
        crossCovariance += weightsCovariance(i) * (stateDiff * measurementDiff);
    }

    return crossCovariance;
}

std::pair<Eigen::Vector4d, RMMatrix4x4d> UKFBase::updateAccelerometer(const double measurement, const double t_current)
{
    const double adjustedMeasurement = measurement + this->getGravity(this->x(0));

    this->t_last_update = t_current;

    RMMatrix4x9d sigmaPoints;
    RMVector9x1d weightsMean;
    RMVector9x1d weightsCovariance;

    std::tie(sigmaPoints, weightsMean, weightsCovariance) = this->generateSigmaPoints(this->x, this->P);

    const RMVector1x9d predictedMeasurementSigmaPoints = this->measurementModelAccelerometer(sigmaPoints);

    const double predictedMeasurement = predictedMeasurementSigmaPoints * weightsMean;

    const double innovationCovariance = this->calculateCovariance(predictedMeasurementSigmaPoints, predictedMeasurement, weightsCovariance, this->R_acc);
    const Eigen::Vector4d crossCovariance = this->calculateCrossCovariance(sigmaPoints, this->x, predictedMeasurementSigmaPoints, predictedMeasurement, weightsCovariance);

    const Eigen::Vector4d kalmanGain = crossCovariance / innovationCovariance;

    const double innovation = adjustedMeasurement - predictedMeasurement;

    Eigen::Vector4d updatedState = this->x + kalmanGain * innovation;
    RMMatrix4x4d updatedCovariance = this->P - kalmanGain * innovationCovariance * kalmanGain.transpose();

    this->x = updatedState;
    this->P = updatedCovariance;

    return {updatedState, updatedCovariance};
}