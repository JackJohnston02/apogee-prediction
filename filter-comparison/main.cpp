#include <eigen3/Eigen/Dense>
#include <memory>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <vector>
#include <utility>   
#include <chrono>    

#include "KalmanFilter.h"
#include "CKFConstantAcceleration.h"
#include "CKFConstantCB.h"
#include "EKFConstantAcceleration.h"
#include "EKFConstantCB.h"
#include "UKFConstantAcceleration.h"
#include "UKFConstantCB.h"
#include "RowMajorMatrix.h"

typedef struct flightData
{
    std::vector<double> timestamp;
    std::vector<double> baro_altitude;
    std::vector<double> imu_accZ;

    void normaliseTimestamps()
    {
        if (timestamp.empty())
        {
            return;
        }

        const double minTimestamp = *std::min_element(timestamp.begin(), timestamp.end());

        for (size_t i = 0; i < timestamp.size(); i++)
        {
            timestamp[i] -= minTimestamp;
        }
    }

    void tareBarometer()
    {
        if (baro_altitude.empty())
        {
            return;
        }

        const double initialBaro = baro_altitude[0];

        for (size_t i = 0; i < baro_altitude.size(); i++)
        {
            baro_altitude[i] -= initialBaro;
        }
    }

    void cropDataStruct(const double startTimestamp, const double endTimestamp)
    {
        std::vector<double> croppedTimestamp;
        std::vector<double> croppedBaroAltitude;
        std::vector<double> croppedImuAccZ;

        for (size_t i = 0; i < timestamp.size(); i++)
        {
            const double currentTimestamp = timestamp[i];
            if (currentTimestamp >= startTimestamp && currentTimestamp <= endTimestamp)
            {
                croppedTimestamp.push_back(currentTimestamp);
                croppedBaroAltitude.push_back(baro_altitude[i]);
                croppedImuAccZ.push_back(imu_accZ[i]);
            }
        }

        timestamp = std::move(croppedTimestamp);
        baro_altitude = std::move(croppedBaroAltitude);
        imu_accZ = std::move(croppedImuAccZ);
    }

    void filter(const double startTimestamp, const double endTimestamp)
    {
        normaliseTimestamps();
        tareBarometer();
        cropDataStruct(startTimestamp, endTimestamp);
    }

} flight_data_t;

double calculateMeanPercentageError(std::vector<std::pair<double, double>> apogees)
{
    const double finalApogee = apogees.at(apogees.size()-1).first;

    double meanApogee = 0.0;

    for (std::pair apogeePair : apogees)
    {
        meanApogee += apogeePair.first;
    }

    meanApogee /= apogees.size();

    const double error = std::abs(meanApogee - finalApogee);

    return (error / finalApogee) * 100;
}

std::vector<std::pair<double, double>> runFilter(KalmanFilter* filter, const flight_data_t& flightData, const double apogeeTime, const double dt)
{
    double z_b = flightData.baro_altitude[0];
    double z_a = flightData.imu_accZ[0];
    int k = 0;
    double t = 0;

    std::vector<std::pair<double, double>> predictions; // apogee, time

    while (t < apogeeTime)
    {
        t += dt;

        filter->predict(t);

        if (k < static_cast<int>(flightData.timestamp.size()))
        {
            if (flightData.baro_altitude[k] != z_b)
            {
                z_b = flightData.baro_altitude[k];
                filter->updateBarometer(z_b, t);
            }

            if (flightData.imu_accZ[k] != z_a)
            {
                z_a = flightData.imu_accZ[k];
                filter->updateAccelerometer(z_a, t);
            }

            k++;
        }

        predictions.push_back({filter->getApogee().first, t});
    }

    return predictions;
}

flight_data_t readFlightDataFromCSV(const std::string& filename);

int main(int argc, char const *argv[])
{
    const double measurement_sigma_bar = 0.5744578867366569;   // Barometer measurement STD
    const double measurement_sigma_acc = 0.006942717204787825; // Accelerometer measurement STD
    const double sigma_Q = 0.2;
    const double sigma_Q_Cb = 2;

    flight_data_t flightData = readFlightDataFromCSV("regulus.csv");
    const double apogeeTime = 27;
    const double startTime = 0;
    const double endTime = 30000;
    flightData.filter(startTime, endTime);

    const double initialT = 0;

    Eigen::Vector4d initialState(flightData.baro_altitude[0], 0, 0, 1000);
    RMMatrix4x4d initialCovariance = 0.01 * RMMatrix4x4d::Identity();

    std::vector<std::pair<std::string, std::unique_ptr<KalmanFilter>>> filters;
    filters.emplace_back("EKF Constant Acceleration", std::make_unique<EKFConstantAcceleration>(initialState, initialCovariance, sigma_Q, sigma_Q_Cb, measurement_sigma_acc, measurement_sigma_bar, initialT));
    filters.emplace_back("EKF Constant CB", std::make_unique<EKFConstantCB>(initialState, initialCovariance, sigma_Q, sigma_Q_Cb, measurement_sigma_acc, measurement_sigma_bar, initialT));
    filters.emplace_back("UKF Constant Acceleration", std::make_unique<UKFConstantAcceleration>(initialState, initialCovariance, sigma_Q, sigma_Q_Cb, measurement_sigma_acc, measurement_sigma_bar, initialT));
    filters.emplace_back("UKF Constant CB", std::make_unique<UKFConstantCB>(initialState, initialCovariance, sigma_Q, sigma_Q_Cb, measurement_sigma_acc, measurement_sigma_bar, initialT));
    filters.emplace_back("CKF Constant Acceleration", std::make_unique<CKFConstantAcceleration>(initialState, initialCovariance, sigma_Q, sigma_Q_Cb, measurement_sigma_acc, measurement_sigma_bar, initialT));
    filters.emplace_back("CKF Constant CB", std::make_unique<CKFConstantCB>(initialState, initialCovariance, sigma_Q, sigma_Q_Cb, measurement_sigma_acc, measurement_sigma_bar, initialT));

    std::cout << std::left << std::setw(30) << "Filter"
              << std::setw(20) << "Final Apogee (m)"
              << std::setw(25) << "Mean Percentage Error"
              << std::setw(20) << "Execution Time (ms)" << std::endl;
    std::cout << std::string(95, '-') << std::endl;

    for (const std::pair<std::string, std::unique_ptr<KalmanFilter>>& filterPair : filters)
    {
        auto startTime = std::chrono::high_resolution_clock::now();

        std::vector<std::pair<double, double>> apogeeResults = runFilter(filterPair.second.get(), flightData, apogeeTime, 0.01);
        const double finalApogee = apogeeResults.at(apogeeResults.size()-1).first;
        const double meanPercentageError = calculateMeanPercentageError(apogeeResults);

        auto endTime = std::chrono::high_resolution_clock::now();

        const std::chrono::duration<double, std::milli> elapsed = endTime - startTime;
        const double elapsedTimeMS = elapsed.count();

        std::cout << std::left << std::setw(30) << filterPair.first
                  << std::setw(20) << finalApogee
                  << std::setw(25) << meanPercentageError
                  << std::setw(20) << elapsedTimeMS << std::endl;
    }

    return 0;
}

flight_data_t readFlightDataFromCSV(const std::string& filename)
{
    flight_data_t flightData;

    std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Could not open file " << std::endl;
        return flightData;
    }

    std::string line;
    if (!std::getline(file, line))
    {
        std::cerr << "Cannot read header line" << std::endl;
        return flightData;
    }

    std::vector<std::string> columnNames;
    std::stringstream ssHeader(line);
    std::string columnName;
    while (std::getline(ssHeader, columnName, ','))
    {
        columnNames.push_back(columnName);
    }

    int timestampIndex = -1;
    int baroAltitudeIndex = -1;
    int imuAccZIndex = -1;

    for (size_t i = 0; i < columnNames.size(); ++i)
    {
        if (columnNames[i] == "timestamp")
        {
            timestampIndex = static_cast<int>(i);
        }
        else if (columnNames[i] == "baro_altitude")
        {
            baroAltitudeIndex = static_cast<int>(i);
        }
        else if (columnNames[i] == "imu_accZ")
        {
            imuAccZIndex = static_cast<int>(i);
        }
    }

    if (timestampIndex == -1 || baroAltitudeIndex == -1 || imuAccZIndex == -1)
    {
        std::cerr << "Could not find all columns" << std::endl;
        return flightData;
    }

    while (std::getline(file, line))
    {
        std::stringstream ss(line);
        std::string value;
        std::vector<std::string> values;
        while (std::getline(ss, value, ','))
        {
            values.push_back(value);
        }

        if (values.size() != columnNames.size())
        {
            std::cerr << "Data row does not match header columns." << std::endl;
            continue;
        }

        try
        {
            double timestampValue = std::stod(values[timestampIndex]);
            double baroAltitudeValue = std::stod(values[baroAltitudeIndex]);
            double imuAccZValue = std::stod(values[imuAccZIndex]);

            flightData.timestamp.push_back(timestampValue);
            flightData.baro_altitude.push_back(baroAltitudeValue);
            flightData.imu_accZ.push_back(imuAccZValue);
        }
        catch (const std::exception& e)
        {
            std::cerr << "Error parsing line: " << e.what() << std::endl;
            continue;
        }
    }

    file.close();

    return flightData;
}