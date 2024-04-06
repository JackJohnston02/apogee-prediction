classdef Navigation_Algorithm
    %NAVIGATION_ALGORITHM Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        filters
        x           % Navigation State Vector
        q           % Navigation Quaternion (body to NED)
        b           % Gyroscope Biases
        y           % Struct of latest measurements
        Pt          % Error Covariance of translational filter
        Pa          % Error Covariance of attitude filter
    end
    
    methods
        function obj = Navigation_Algorithm(xi,Pt,Pa,sigma,Ts,q,settingsCA_KF,settings_MEKF)
            %NAVIGATION_ALGORITHM Construct an instance of this class
            %   Instantiate a Navigation_Algorithm class that consists of a
            %   linear NED 3D Kalman filter and a Quaternion attitude
            %   estimator

            % Declare Linear CA Kalman Filter
            obj.filters.translationKF = CA_KF(xi(1:6),Pt,q,sigma,Ts,settingsCA_KF);

            % Declare Attitude Estimation Filter
            obj.filters.attitudeKF = MEKF(xi(7:13),Pa,sigma,Ts,settings_MEKF);

        end
        
        function obj = updateNavigationStates(obj,y_acc,y_gps,y_gyr,y_alt,y_mag)
            %UPDATENAVIGATIONSTATES Updates the filters
            %   Perform time updates and, if measurements are available,
            %   measurement updates of the Navigation Algorithms filters

            %% Time Update
            % We assume that the Navigation Algorithm runs at the frequency
            % of the IMU Accelerometer and Gyroscope measurements.

            % Translational Filter Prediction
            obj.filters.translationKF = obj.filters.translationKF.propagate(obj.filters.translationKF,obj.q,y_acc);

            % Attitude Filter Prediction
            obj.filters.attitudeKF = obj.filters.attitudeKF.propagate(obj.filters.attitudeKF,y_gyr);
            
            %% Filter Measurement Update
            % GPS Correction Step
            if ~isempty(y_gps)
                % Translational Filter Correction
                obj.filters.translationKF = obj.filters.translationKF.updateGPS(obj.filters.translationKF,obj.q,y_gps);
            end

            % Altimeter Correction Step
            if ~isempty(y_alt)
                % Translational Filter Correction
                obj.filters.translationKF = obj.filters.translationKF.updateAltimeter(obj.filters.translationKF,y_alt);
            end

            % Magmetometer Correction Step
            if ~isempty(y_gps)
                % Attitude Filter Update using Magnetometer
                obj.filters.attitudeKF = obj.filters.attitudeKF.updateMagnetometer(obj.filters.attitudeKF,y_mag,r_mag);
            end

            % Accelerometer Correction Step
            if ~isempty(y_gps) && obj.filters.attitudeKF.useAccUpdate
                % Attitude Filter Update using Accelerometer
                obj.filters.attitudeKF = obj.filters.attitudeKF.updateMagnetometer(obj.filters.attitudeKF,y_acc);
            end

            %% Update Nav States for use by other objects
            % Translational States
            obj.x = obj.filters.translationKF.xkk;
            obj.Pt = obj.filters.translationKF.Pkk;

            % Attitude States
            obj.q = obj.filters.attitudeKF.q;
            obj.b = obj.filters.attitudeKF.b;
            obj.Pa = obj.filters.attitudeKF.Pkk;

        end
    end
end

