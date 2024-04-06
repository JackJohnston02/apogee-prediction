classdef CA_KF < Kalman_Filter
    %CA_KF Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods
        function obj = CA_KF(xi,Pi,q,sigma,Ts,settings)
            %CA_KF Construct an instance of this class
            %   Detailed explanation goes here
            obj.xk_1k_1 = xi;
            obj.Pk_1k_1 = Pi;
            Fr = [1 Ts 0.5*Ts^2; 0 1 Ts; 0 0 1];
            obj.F = blkdiag(Fr,Fr,Fr);
            Qr = q*[Ts^5/20 Ts^4/8 Ts^3/6; Ts^4/8 Ts^3/3 Ts^2/2; Ts^3/6 Ts^2/2 Ts];
            obj.Q = blkdiag(Qr,Qr,Qr);
            obj.R.Ralt = sigma.sigma_alt^2;
            % GPS might have different noise properties for altitude and
            % X/Y
            obj.R.Rgps = blkdiag(sigma.sigma_gps(1)^2,sigma.sigma_gps(2),sigma.sigma_gps(3));
            obj.R.Racc = sigma.sigma_acc^2*eye(3);

            %% Measurement matrices for each measurement type
            % Accelerometer
            obj.H.Hacc = [zeros(1,6) 1 0 0; zeros(1,7) 1 0; zeros(1,8) 1];
            % Altimeter
            obj.H.Hgps = [0 0 1 zeros(1,6)];

            obj.I = eye(9);

            obj.settings.useGPSAltitude = settings.useGPSAltitude;
                        % GPS
            if obj.settings.useGPSAltitude
                obj.H.Hgps = [1 0 0 zeros(1,6); 0 1 0 zeros(1,6); 0 0 1 zeros(1,6)];
            else
                obj.H.Hgps = [1 0 0 zeros(1,6); 0 1 0 zeros(1,6)];
            end
        end
        
        function obj = propagate(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here

            % Perform Time Update using latest estimates of NED R^3
            % Kinematics
            obj.xkk_1 = obj.F*obj.xk_1k_1;
            

            % Propagate Covariance
            obj.Pkk_1 = obj.F*obj.Pk_1k_1*obj.F' + obj.Q;
        end

        function obj = updateAccelerometer(obj,q,y_acc)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here

            % Rotate Body Acceleration measurements into NED frame using
            % current estimate of q_ned2b
            q_b2ned = quatconj(q')';
            acc_n = quatrotate(q_b2ned,y_acc')';

            % Account for Gravitational Acceleration (acting in +D) using
            % latest value of g
            acc_nr = acc_n + obj.g;
            
            % Calculate Kalman Gain
            K = obj.Pkk_1*obj.H.Hacc'/(obj.H.Hacc*obj.Pkk_1*obj.H.Hacc' + obj.R.Racc);

            % Update State Estimate
            obj.xkk = obj.xkk_1 + K*(acc_nr - obj.H.Hacc*obj.xkk_1);

            % Update Covariance using Joseph Form
            obj.Pkk = (obj.I - K*obj.H.Hacc)*obj.Pkk_1*(obj.I - K*obj.H.Hacc)' + K*obj.R.Racc*K';
        end

        function obj = updateGPS(obj,y_gps)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here

            % NOTE: Since the filter will run at the same sample rate as
            % the accelerometer, the updateGPS function wil use the
            % posterior state estimate xkk rather than xkk_1

            % Perform update depending on whether GPS altitude is used.
            
            % Calculate Kalman Gain
            K = obj.Pkk*obj.H.Hgps'/(obj.H.Hgps*obj.Pkk*obj.H.Hgps' + obj.R.Rgps);

            % Update State Estimate
            obj.xkk = obj.xkk + K*(y_gps - obj.H.Hgps*obj.xkk);

            % Update Covariance using Joseph Form
            obj.Pkk = (obj.I - K*obj.H.Hgps)*obj.Pkk_1*(obj.I - K*obj.H.Hgps)' + K*obj.R.Rgps*K';
        end

        function obj = updateAltimeter(obj,y_alt)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here

            % NOTE: Since the filter will run at the same sample rate as
            % the accelerometer, the updateAltimeter function wil use the
            % posterior state estimate xkk rather than xkk_1

            % Calculate Kalman Gain
            K = obj.Pkk*obj.H.Halt'/(obj.H.Halt*obj.Pkk_1*obj.H.Halt' + obj.R.Ralt);

            % Update State Estimate
            obj.xkk = obj.xkk + K*(y_alt - obj.H.Halt*obj.xkk_1);

            % Update Covariance using Joseph Form
            obj.Pkk = (obj.I - K*obj.H.Halt)*obj.Pkk_1*(obj.I - K*obj.H.Halt)' + K*obj.R.Ralt*K';
        end
    end
end

