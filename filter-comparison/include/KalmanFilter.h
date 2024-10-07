#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include "RowMajorMatrix.h"

class KalmanFilter
{
public:
    KalmanFilter(const Eigen::Vector4d& initialState, const RMMatrix4x4d& initialCovariance, const double& sigma_Q,
                 const double& sigma_Q_Cb, const double& measurement_sigma_acc, const double& measurement_sigma_bar,
                 const double& t);

    virtual void predict(const double t_current) = 0;
    virtual std::pair<Eigen::Vector4d, RMMatrix4x4d> updateAccelerometer(const double measurement, const double t_current) = 0;
    virtual std::pair<Eigen::Vector4d, RMMatrix4x4d> updateBarometer(const double measurement, const double t_current) = 0;
    virtual std::pair<double, double> getApogee() = 0;

    RMMatrix4x4d calculateProcessNoise(const double& dt);
    double getDensity(const double height);
    double getGravity(const double height);

    const double p_0 = 101325;   // Standard sea level atmospheric pressure
    const double M = 0.0289652;  // Molar mass of dry air
    const double R = 8.31445;    // Ideal gas constant
    const double T_0 = 288.15;   // Standard sea level temperature
    const double L = 0.0065;     // Temperature lapse rate
    const double R_e = 6371000;  // Earth radius
    const double g_0 = -9.80665; // Standard gravity

protected:
    Eigen::Vector4d x;
    RMMatrix4x4d P;
    RMMatrix4x4d Q;
    double R_acc;
    double R_bar;
    double t_last_update;
    double sigma_Q;
    double sigma_Q_Cb;

    const double dt_apa = 0.01;
};

#endif // KALMAN_FILTER_H