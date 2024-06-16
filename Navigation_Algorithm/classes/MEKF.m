classdef MEKF < Kalman_Filter
    %MEKF Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        q
        b
        omega
    end
    
    methods
        function obj = MEKF(xi,Pi,sigma,Ts,settings)
            %MEKF Construct an instance of this class
            %   Detailed explanation goes here
            obj.xkk = xi;
            obj.Pkk = Pi;
            Qomega = sigma.sigma_gyr^2*eye(3);
            Qbias = sigma.sigma_bias^2*eye(3);
            obj.Q = blkdiag(Qomega,Qbias);
            obj.R.Rmag = sigma.sigma_mag^2*eye(3);
            % Option to use Accelerometer in future (body acceleration
            % measurements
            obj.R.Racc = sigma.sigma_acc^2*eye(3);

            %% Propagation matrices (LIEKF on SO(3)xR^3
            obj.F = [-eye(3) -eye(3)*Ts; zeros(3,3) eye(3)];
            obj.G = [-eye(3) zeros(3,3); zeros(3,3) eye(3)];
            
            obj.I = eye(6);
            obj.Ts = Ts;

            obj.settings = settings;

            obj.q = obj.xkk(1:4);
            obj.b = obj.xkk(5:7);

        end
        
        function obj = propagate(obj,y_omega)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            
            % Calculate estimated angular rate
            obj.omega = y_omega - obj.b;
            
            obj.q = quatmultiply(obj.q',s3_expmap(obj.omega*obj.Ts/2)')';

            % Assume constant velocity bias
            obj.b = obj.b;

            % Form state
            obj.xkk = [obj.q; obj.b];

            % LIEKF F
            obj.F = [-so3_wedge(obj.omega) -eye(3); zeros(3,3) zeros(3,3)];

            % Propagate using EKF (simplification of a Left-Invariant EKF)
            obj.Pkk = obj.F*obj.Pkk*obj.F' + obj.G*obj.Q*obj.G';

        end

        function obj = updateMagnetometer(obj,y_mag,r_mag)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            
            % Compute normalised estimated reference and measurement vectors
            r_mag = r_mag/norm(r_mag);
            y_mag = y_mag/norm(y_mag);

            % Rotate reference NED Magnetometer vector into body frame
            % (LIEKF Update) could also update in global frame (RIEKF)
            r_mag_b = quatrotate(obj.q',r_mag)';

            % Update Measurement Jacobian (LIEKF Jacobian)
            obj.H.Hmag = [so3_wedge(r_mag_b) zeros(3,3)];

            % Calculate the Kalman Gain
            K = obj.Pkk*obj.H.Hmag'/(obj.H.Hmag*obj.Pkk*obj.H.Hmag' + obj.R.Rmag);

            % Update state using MEKF Update (LIEKF local frame)
            dx = K*(y_mag - r_mag_b);

            % Construct quaternion from so3 Lie algebra
            qe = [sqrt(1 - 0.25*dx(1:3)'*dx(1:3)); 0.5*dx(1:3)];

            obj.q = quatmultiply(obj.q',qe')';

            % Normalise quaternion (remove in future)
            obj.q = quatnormalize(obj.q')';
            
            % Correct Gyroscope biases
            obj.b = obj.b + dx(4:6);

            % Form posterior state
            obj.xkk = [obj.q; obj.b];
            
            % Update Covariance using Joseph Form
            obj.Pkk = (obj.I - K*obj.H.Hmag)*obj.Pkk*(obj.I - K*obj.H.Hmag)' + K*obj.R.Rmag*K';

        end

        function obj = updateAccelerometer(obj,y_acc,a_ned_est)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here

            % Gravity vector points along D+ vector i.e. g = [0 0 1]';
            r_grav = [0 0 -1]';

            % Subtract NED Acceleration Estimate
            a_b_est = quatrotate(quatconj(obj.q'),a_ned_est')';
            y_acc = y_acc - a_b_est;
            
            % Compute normalised measurement vectors in body frame
            y_acc = y_acc/norm(y_acc);
            
            % Rotate reference NED acceleration vector into body frame
            % (LIEKF Update)
            r_grav_b = quat2dcm(obj.q')*r_grav;

            % Update Measurement Jacobian
            obj.H.Hacc = [so3_wedge(r_grav_b) zeros(3,3)];

            % Calculate the Kalman Gain
            K = obj.Pkk*obj.H.Hacc'/(obj.H.Hacc*obj.Pkk*obj.H.Hacc' + obj.R.Racc);

            % Update state using MEKF Update
            dx = K*(y_acc - r_grav_b);

            % Construct quaternion from so3 Lie algebra
            qe = [sqrt(1 - 0.25*dx(1:3)'*dx(1:3)); 0.5*dx(1:3)];

            obj.q = quatmultiply(obj.q',qe')';

            % Normalise quaternion (remove in future)
            obj.q = quatnormalize(obj.q')';
            
            % Correct Gyroscope biases
            obj.b = obj.b + dx(4:6);

            % Form posterior state
            obj.xkk = [obj.q; obj.b];
            
            % Update Covariance using Joseph Form
            obj.Pkk = (obj.I - K*obj.H.Hacc)*obj.Pkk*(obj.I - K*obj.H.Hacc)' + K*obj.R.Racc*K';

        end
    end
end

