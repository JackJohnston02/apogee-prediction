classdef RIEKF
    %RIEKF Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = public)
        Xk_1k_1
        Zk_1k_1
        Xkk_1
        Zkk_1
        Xkk
        Zkk
        Rk_1k_1
        Rkk_1
        Rkk
        pk_1k_1
        pkk_1
        pkk
        vk_1k_1
        vkk_1
        vkk
        bak_1k_1
        bakk_1
        bakk
        bgk_1k_1
        bgkk_1
        bgkk
        Pk_1k_1
        Pkk_1
        Pkk
        At
        Q
        Qadj
        Qt
        Qg
        Qgb
        Qa
        Qab
        Rgnss
        Rmag
        Ralt
        R
        dt
        n
        m
        Hgnss
        Halt
        Hmag
        Hstack
    end
    properties (Access = protected)
        % Gravity in NED frame
        g = [0 0 9.81]';
    end
    
    methods
        function obj = RIEKF(X0,Z0,P0,Qgyro,Qgyrob,Qacc,Qaccb,Rgnss,Ralt,Rmag,dt)
            %RIEKF Construct an instance of this class
            %   Detailed explanation goes here
            obj.Xk_1k_1 = X0;
            obj.Zk_1k_1 = Z0;
            obj.Pk_1k_1 = P0;

            obj.Qg = Qgyro;
            obj.Qgb = Qgyrob;
            obj.Qa = Qacc;
            obj.Qab = Qaccb;

            obj.Rgnss = Rgnss;
            % Transform altimeter measurement into position measurement
            % with known x,y components
            obj.Ralt = blkdiag(0,0,Ralt);
            obj.Rmag = Rmag;

            % Assume position is entirely driven by acceleration process
            % noise in velocity
            obj.Q = blkdiag(obj.Qg,obj.Qa,zeros(3,3),obj.Qgb,obj.Qab);

            obj.dt = dt;

            obj.n = size(P0,1);
            obj.m = size(obj.Rgnss) + size(obj.Ralt) + size(obj.Rmag);


           
        end
        
        function obj = propagate(obj,omega_bib,f_bib)
            
            % Extract current states from SE_2(3) state element
            obj.Rk_1k_1 = obj.Xk_1k_1(1:3,1:3);
            obj.vk_1k_1 = obj.Xk_1k_1(1:3,4);
            obj.pk_1k_1 = obj.Xk_1k_1(1:3,5);

            % Extract current state from R^6 vector element
            obj.bgk_1k_1 = obj.Zk_1k_1(1:3,1);
            obj.bak_1k_1 = obj.Zk_1k_1(4:6,1);
            
            %% State Propagation
            % Propagate system using discretised system equations using a 
            % zero-order hold and Euler integration
            % Attitude
            obj.Rkk_1 = obj.Rk_1k_1*so3_expmap((omega_bib - obj.bgk_1k_1)*obj.dt);
            % NED Velocity
            obj.vkk_1 = obj.vk_1k_1 + obj.Rk_1k_1*(f_bib - obj.bak_1k_1)*obj.dt + obj.g*obj.dt;
            % NED Position
            obj.pkk_1 = obj.pk_1k_1 + obj.vk_1k_1*obj.dt + 0.5*obj.Rk_1k_1*(f_bib - obj.bak_1k_1)*obj.dt^2 + 0.5*obj.g*obj.dt^2;

            % Gyroscope and Accelerometer biases are assumed to be constant
            obj.bgkk_1 = obj.bgk_1k_1;
            obj.bakk_1 = obj.bak_1k_1;

            % Form States for logging
            obj.Xkk_1 = [obj.Rkk_1 obj.vkk_1 obj.pkk_1;...
                         zeros(2,3) eye(2)];
            obj.Zkk_1 = [obj.bgkk_1; obj.bakk_1];

            %% Covariance Propagation
            % First-order approximation of the algebraic Ricatti equation
            obj = evaluateStatePropagationMatrix(obj);
            Phi = expm(obj.At*obj.dt);

            % Evaluate process noise covariance on the group
            obj = evaluateProcessNoiseCovarianceMatrix(obj);
            Qk = Phi*obj.Qt*Phi'*obj.dt;

            % Propagate covariance
            obj.Pkk_1 = Phi*obj.Pk_1k_1*Phi' + Qk;

        end

        function obj = updateGNSS(obj,y_gnss)
            % Extract current states from SE_2(3) state element
            obj.Rkk_1 = obj.Xkk_1(1:3,1:3);
            obj.vkk_1 = obj.Xkk_1(1:3,4);
            obj.pkk_1 = obj.Xkk_1(1:3,5);

            % Extract current state from R^6 vector element
            obj.bgkk_1 = obj.Zkk_1(1:3,1);
            obj.bakk_1 = obj.Zkk_1(4:6,1);
            
            %% GNSS Measurement is a Left-Invariant Observation
            % Evaluate Jacobian
            obj.Hgnss = [zeros(3) zeros(3) eye(3) zeros(3) zeros(3)];
            
            % Transform Right-Invariant Covariance to Left-Invariant
            % Covariance
            AdjX = blkdiag(se2_3_adjoint(obj.Xkk_1),eye(6));
            leftPkk_1 = AdjX*obj.Pkk_1*AdjX';

            % Rotate Measurement Noise Covariance
            Ngnss = obj.Rkk_1'*obj.Rgnss*obj.Rkk_1;

            % Calculate Kalman Gain
            K = leftPkk_1*obj.Hgnss'*(obj.Hgnss*leftPkk_1*obj.Hgnss' + Ngnss)^(-1);
            
            % Form right-invariant observation equation
            Ygnss = [y_gnss; 0; 1];
            b = [zeros(4,1); 1];

            % Auxilliary gain matrix (as in Contact-aided...)
            Pi = [eye(3) zeros(3,2)];

            % Compute innovation
            nu = K*Pi*(obj.Xkk_1^(-1)*Ygnss - b);
            dX = nu(1:9,1);
            dZ = nu(10:15,1);

            % Update state using left multipplication
            obj.Xkk = obj.Xkk_1*se2_3_expmap(dX);
            obj.Zkk = obj.Zkk_1 + dZ;

            % Update Covariance using Joseph form
            leftPkk = (eye(obj.n) - K*obj.Hgnss)*leftPkk_1*(eye(obj.n) - K*obj.Hgnss)' + K*Ngnss*K';

            % Transform Left-Covariance to Right Covariance
            AdjX = blkdiag(se2_3_adjoint(obj.Xkk),eye(6));
            obj.Pkk = AdjX*leftPkk*AdjX';


        end

        function obj = updateAlt(obj,y_alt)
            %Ref: https://www.youtube.com/watch?v=WHBgSqRPpu4
            % Extract current states from SE_2(3) state element
            obj.Rkk_1 = obj.Xkk_1(1:3,1:3);
            obj.vkk_1 = obj.Xkk_1(1:3,4);
            obj.pkk_1 = obj.Xkk_1(1:3,5);

            % Extract current state from R^6 vector element
            obj.bgkk_1 = obj.Zkk_1(1:3,1);
            obj.bakk_1 = obj.Zkk_1(4:6,1);

            % Evaluate Jacobian
            obj.Halt = [zeros(3,6) -eye(3) zeros(3,6)];
            
            % Rotate Measurement Noise Covariance
            Nalt = obj.Rkk_1*obj.Ralt*obj.Rkk_1';

            % Calculate Kalman Gain
            K = obj.Pkk_1*obj.Halt'*(obj.Halt*obj.Pkk_1*obj.Halt' + Nalt)^(-1);
            
            % Form right-invariant observation equation
            % We need to format the altimeter measurements into the form
            % for the RIEKF. We use the current position estimate and
            % replace the down component with the altitude, before rotating
            % back into the body frame
            Yalt = [-obj.Rkk_1'*[obj.pkk_1(1); obj.pkk_1(2); -y_alt]; 0; 1];
            b = zeros(5,1);
            Pi = [eye(3) zeros(3,2)];

            % Compute innovation
            temp = K*Pi*(obj.Xkk_1*Yalt - b);
            dX = temp(1:9,1);
            dZ = temp(10:15,1);

            % Update state using right-multiplication
            obj.Xkk = se2_3_expmap(dX)*obj.Xkk_1;
            obj.Zkk = obj.Zkk_1 + dZ;

            % Update Covariance using Joseph form
            obj.Pkk = (eye(obj.n) - K*obj.Halt)*obj.Pkk_1*(eye(obj.n) - K*obj.Halt)' + K*Nalt*K';
        end

        function obj = updateMag(obj,y_mag,ref_mag)
            % Extract current states from SE_2(3) state element
            obj.Rkk_1 = obj.Xkk_1(1:3,1:3);
            obj.vkk_1 = obj.Xkk_1(1:3,4);
            obj.pkk_1 = obj.Xkk_1(1:3,5);

            % Extract current state from R^6 vector element
            obj.bgkk_1 = obj.Zkk_1(1:3,1);
            obj.bakk_1 = obj.Zkk_1(4:6,1);

            % Evaluate Jacobian
            obj.Hmag = [so3_wedge(ref_mag) zeros(3) zeros(3) zeros(3) zeros(3)];
            % obj.Hmag = [so3_wedge(y_mag) zeros(3) zeros(3) zeros(3) zeros(3)];
            
            % Rotate Measurement Noise Covariance
            Nmag = obj.Rkk_1*obj.Rmag*obj.Rkk_1';

            % Calculate Kalman Gain
            K = obj.Pkk_1*obj.Hmag'*(obj.Hmag*obj.Pkk_1*obj.Hmag' + Nmag)^(-1);
            
            % Form right-invariant observation equation
            Ymag = [y_mag; 0; 0];
            b = [ref_mag/norm(ref_mag); 0; 0];
            Pi = [eye(3) zeros(3,2)];

            % Compute innovation
            temp = K*Pi*(obj.Xkk_1*Ymag - b);
            dX = temp(1:9,1);
            dZ = temp(10:15,1);

            % Update state using right-multiplication
            obj.Xkk = se2_3_expmap(dX)*obj.Xkk_1;
            obj.Zkk = obj.Zkk_1 + dZ;

            % Update Covariance using Joseph form
            obj.Pkk = (eye(obj.n) - K*obj.Hmag)*obj.Pkk_1*(eye(obj.n) - K*obj.Hmag)' + K*Nmag*K';
        end

        function obj = updateStacked(obj,y_gnss,y_alt,y_mag,ref_mag)
            % Extract current states from SE_2(3) state element
            obj.Rkk_1 = obj.Xkk_1(1:3,1:3);
            obj.vkk_1 = obj.Xkk_1(1:3,5);
            obj.pkk_1 = obj.Xkk_1(1:3,4);

            % Extract current state from R^6 vector element
            obj.bgkk_1 = obj.Zkk_1(1:3,1);
            obj.bakk_1 = obj.Zkk_1(4:6,1);

            % Evaluate Jacobian
            obj.Hgnss = [zeros(3) zeros(3) -eye(3) zeros(3) zeros(3)];
            obj.Halt = [zeros(3) zeros(3) -eye(3) zeros(3) zeros(3)];
            obj.Hmag = [so3_wedge(ref_mag) zeros(3) zeros(3) zeros(3) zeros(3)];
            obj.Hstack = [obj.Hgnss; obj.Halt; obj.Hmag];
            
            % Rotate Measurement Noise Covariance
            Ngnss = obj.Rkk_1*obj.Rgnss*obj.Rkk_1';
            Nalt = obj.Rkk_1*obj.Ralt*obj.Rkk_1';
            Nmag = obj.Rkk_1*obj.Rmag*obj.Rkk_1';
            Nstack = blkdiag(Ngnss,Nalt,Nmag);

            % Calculate Kalman Gain
            K = obj.Pkk_1*obj.Hstack'*(obj.Hstack*obj.Pkk_1*obj.Hstack' + Nstack)^(-1);
            
            % Form right-invariant observation equation
            Ygnss = [y_gnss; 0; 1];
            Yalt = [-obj.Rkk_1'*[obj.pkk_1(1); obj.pkk_1(2); -y_alt]; 0; 1];
            Ymag = [y_mag; 0; 0];
            Ystack = [Ygnss; Yalt; Ymag];

            bmag = [ref_mag; 0; 0];
            bgnss = zeros(5,1);
            balt = zeros(5,1);
            bstack = [bgnss; balt; bmag];

            % Compute innovation
            temp = K*(obj.Xkk_1*Ystack - bstack);
            dX = temp(1:9,1);
            dZ = temp(10:15,1);

            % Update state using right-multiplication
            obj.Xkk = se2_3_expmap(dX)*obj.Xkk_1;
            obj.Zkk = obj.Zkk_1 + dZ;

            % Update Covariance using Joseph form
            obj.Pkk = (eye(obj.n) - K*obj.Hstack)*obj.Pkk_1*(eye(obj.n) - K*obj.Hstack)' + K*obj.Rgnss*K';
        end

        function obj = evaluateStatePropagationMatrix(obj)
            % Function to evaluate the state propagation matrix A for the
            % RIEKF.
            O = zeros(3,3);
            obj.At = [O O O -obj.Rk_1k_1 O;...
                 so3_wedge(obj.g) O O -so3_wedge(obj.vk_1k_1)*obj.Rk_1k_1 -obj.Rk_1k_1;...
                 O eye(3) O -so3_wedge(obj.pk_1k_1)*obj.Rk_1k_1 O;...
                 O O O O O;...
                 O O O O O];
        end
        function obj = evaluateProcessNoiseCovarianceMatrix(obj)
            % Function to evaluate the state propagation matrix A for the
            % RIEKF.
            O = zeros(3,3);
            AdjX = se2_3_adjoint(obj.Xk_1k_1);
            obj.Qadj = zeros(15,15);
            obj.Qadj(1:9,1:9) = AdjX;
            obj.Qadj(10:15,10:15) = eye(6);

            obj.Qt = obj.Qadj*obj.Q*obj.Qadj';
        end
    end
end

