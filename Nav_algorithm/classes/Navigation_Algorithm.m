classdef Navigation_Algorithm
    %NAVIGATION_ALGORITHM Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        filters     % Struct of Kalman filters used for state estimation
                    % TranslationalKF - NED frame cartesian constant
                    % acceleration model
                    % AttitudeKF      - MEKF (Currently!)
        predictor   % Apogee prediction algorithm            
        x           % Navigation State Vector
        q           % Navigation Quaternion (body to NED)
        b           % Gyroscope Biases
        h_ap        % Estimate of apogee
        y           % Struct of latest measurements
        Pt          % Error Covariance of translational filter
        Pa          % Error Covariance of attitude filter
        LLA0        % Initial Geodetic Coordinates
        wgs84       % Earth Model for NED to Geodetic Conversion
        settings
        state

        %% TEST STATES
        trueU
    end
    
    methods
        function obj = Navigation_Algorithm(xi,Pt,Pa,sigma,Ts,q,settingsCA_KF,settings_MEKF,settings_ApPred,LLA0)
            %NAVIGATION_ALGORITHM Construct an instance of this class
            %   Instantiate a Navigation_Algorithm class that consists of a
            %   linear NED 3D Kalman filter and a Quaternion attitude
            %   estimator
            obj.LLA0 = LLA0;
            obj.wgs84 = wgs84Ellipsoid;
            obj.state.burnout = false;

            obj.settings.useApogeePredict = settings_ApPred.usePredict;

            % Declare Linear CA Kalman Filter
            obj.filters.translationKF = CA_KF(xi(1:9),Pt,q,sigma,Ts,settingsCA_KF);

            % Declare Attitude Estimation Filter
            obj.filters.attitudeKF = MEKF(xi(10:16),Pa,sigma,Ts,settings_MEKF);

            % Decalre Apogee Prediction Algorithm
            obj.predictor.apogeePredictor = Apogee_Predictor(settings_ApPred.numParticles,Ts);
            obj.h_ap = 0;

        end
        
        function obj = updateNavigationStates(obj,buffer,qtruened)
            %UPDATENAVIGATIONSTATES Updates the filters
            %   Perform time updates and, if measurements are available,
            %   measurement updates of the Navigation Algorithms filters

            % Extract measurements from the buffer


            %% Time Update
            % We assume that the Navigation Algorithm runs at the frequency
            % of the IMU Accelerometer and Gyroscope measurements.

            % Translational Filter Prediction
            if isnan(any(buffer.y_acc)) && obj.filters.translationKF.settings.propagateWithAccelerometer
                obj.filters.translationKF = obj.filters.translationKF.propagateAccelerometer;
            else
                obj.filters.translationKF = obj.filters.translationKF.propagate;
            end

            % Attitude Filter Prediction
            obj.filters.attitudeKF = obj.filters.attitudeKF.propagate(buffer.y_omega);
            
            %% Filter Measurement Update
            % Accelerometer Correction Step
            if ~isnan(buffer.y_acc)
                % Translational Filter Correction
                %obj.filters.translationKF = obj.filters.translationKF.updateAccelerometer(obj.filters.attitudeKF.q,buffer.y_acc);
                obj.filters.translationKF = obj.filters.translationKF.updateAccelerometer(qtruened,buffer.y_acc);
            end
            % GPS Correction Step
            if ~isnan(buffer.y_gnss)
                % Need to do PDF transformation to alter the statistics
                % Convert GNSS Measurement from Geodetic to NED
                [xn,yn,zn] = geodetic2ned(buffer.y_gnss(1),buffer.y_gnss(2),buffer.y_gnss(3),obj.LLA0(1),obj.LLA0(2),obj.LLA0(3),obj.wgs84);
                if obj.filters.translationKF.settings.useGNSSAltitude
                    y_gnss2 = [xn; yn; zn];
                else
                    y_gnss2 = [xn; yn];
                end
                % Translational Filter Correction
                obj.filters.translationKF = obj.filters.translationKF.updateGPS(y_gnss2);
            end

            % Altimeter Correction Step
            if ~isnan(buffer.y_alt)
                % Translational Filter Correction
                obj.filters.translationKF = obj.filters.translationKF.updateAltimeter(buffer.y_alt);
            end

            % Magmetometer Correction Step
            if ~isnan(buffer.y_mag)
                % Calculate Magnetic Field reference vector based on
                % current NED/Geodetic Position
                [LatEst,LongEst,Aest] = ned2geodetic(obj.filters.translationKF.xkk(1),obj.filters.translationKF.xkk(4),obj.filters.translationKF.xkk(7),obj.LLA0(1),obj.LLA0(2),obj.LLA0(3),obj.wgs84);
                r_mag = wrldmagm(Aest,LatEst,LongEst,decyear(2024,7,4),'2020')';

                % Attitude Filter Update using Magnetometer
                obj.filters.attitudeKF = obj.filters.attitudeKF.updateMagnetometer(buffer.y_mag,r_mag,qtruened);
            end

            % Accelerometer Correction Step
            if ~any(isnan(buffer.y_acc)) && obj.filters.attitudeKF.settings.useAccUpdate
                % Attitude Filter Update using Accelerometer
                a_ned_est = obj.filters.translationKF.xkk([3,6,9],1);
                obj.filters.attitudeKF = obj.filters.attitudeKF.updateAccelerometer(buffer.y_acc,a_ned_est);
            end

            %% Update Nav States for use by other objects
            % Translational States
            obj.x = obj.filters.translationKF.xkk;
            obj.Pt = obj.filters.translationKF.Pkk;

            % Attitude States
            obj.q = obj.filters.attitudeKF.q;
            obj.b = obj.filters.attitudeKF.b;
            obj.Pa = obj.filters.attitudeKF.Pkk;

            %% Update Timestep
            obj.filters.translationKF.xk_1k_1 = obj.filters.translationKF.xkk;
            obj.filters.translationKF.Pk_1k_1 = obj.filters.translationKF.Pkk;
            obj.filters.attitudeKF.xk_1k_1 = obj.filters.attitudeKF.xkk;
            obj.filters.attitudeKF.Pk_1k_1 = obj.filters.attitudeKF.Pkk;

            %% TO-DO - Calculate the updated rho, gravity at current altitude
            g = -9.80665;
            obj.predictor.apogeePredictor.g = g;
            Pap = obj.filters.translationKF.Pkk(7:9,7:9);

            %% TO-DO - State Machine
            if obj.x(9) > 5
                obj.state.burnout = true;
            end

            %% Perform Apogee Prediction based on current state estimates after burnout
            if obj.settings.useApogeePredict && obj.state.burnout
                % obj.predictor.apogeePredictor = obj.predictor.apogeePredictor.predict(-obj.x(7),-obj.x(8),-obj.x(9),Pap);
                obj.predictor.apogeePredictor = obj.predictor.apogeePredictor.predict(obj.trueU(1),obj.trueU(2),obj.trueU(3),Pap);
                obj.h_ap = obj.predictor.apogeePredictor.apogeePred;
            end
        end
    end
end

