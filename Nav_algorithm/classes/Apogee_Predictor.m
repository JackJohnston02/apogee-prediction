classdef Apogee_Predictor
    %APOGEE_PREDICTOR Summary of this class goes here
    %   Detailed explanation goes here

    properties
        apogeePred
        apogeePredHist
        sigmaApogee
        sigmaApogeeHist
        g
        rho
        xp
        T_0
        L
        Ts
        numParticles
        % ANY OTHER states that will be outputted or kept for multiple
        % iterations

    end

    methods
        function obj = Apogee_Predictor(numParticles,Ts)
            %APOGEE_PREDICTOR Construct an instance of this class
            %   Detailed explanation goes here
            obj.apogeePred = 0;
            obj.apogeePredHist = [];
            obj.sigmaApogee = 0;
            obj.sigmaApogeeHist = [];
            obj.numParticles = numParticles;
            
            obj.Ts = Ts;
            obj.g = -9.80665;
            obj.T_0 = 260;
            obj.L = 0.0065;
        end

        function obj = predict(obj,h_est,v_est,a_est,P)
            %METHOD1 Summary of this method goes here
            %   INPUTS:
            %          h_est - current estimate of current altitude
            %          v_est - current estimate of current velocity (Up+)
            %          a_est - current estimate of current acceleration (Up+)
            %          P     - current error covariance of state estimate

            % Perform Apogee Prediction
            %% Generate sample of particles from the posterior mean and covariance of the rocket state
            % Ensure that P is symmetric
            P = (P + P') / 2;

            % Form mean from state estimates
            x = [h_est,v_est,a_est]';

            %For uniform Cc and rho
            %Cc = 2 * (x(3) - g) / (rho * (x(2)*x(2)));
            %rho = 1.225 * (1 - (0.0065 * x(1)) / 288.15)^(-g / (287.05 * 0.0065));
            if obj.numParticles == 1
                obj.xp(:,1) = x;
            else
                obj.xp = mvnrnd(x,P,obj.numParticles)';%Not sure if this is the correct function?
            end

            %% Perform prediction until the velocity prediction is zero
            for i = 1:obj.numParticles
                T = obj.T_0 - obj.L * obj.xp(1,i);
                obj.rho =  1.225 * (1 - (0.0065 * obj.xp(1,i)) / T)^(-obj.g / (287.05 * 0.0065));
                Cc = 2 * (obj.xp(3,i) - obj.g) / (obj.rho * (obj.xp(2,i)^2));
                while obj.xp(2,i) > 0 && obj.xp(1,i) > 0 && obj.xp(1,i) < 5000
                    % Propagate each particle through the prediction algorithm,
                    % constant Cc model
                    T = obj.T_0 - obj.L * obj.xp(1,i);
                    obj.rho =  1.225 * (1 - (0.0065 * obj.xp(1,i)) / T)^(-obj.g / (287.05 * 0.0065));
                    obj.xp(3,i) = obj.g + (0.5 * Cc * obj.rho * obj.xp(2,i)^2);
                    obj.xp(2,i) = obj.xp(2,i) + obj.Ts * obj.xp(3,i);
                    obj.xp(1,i) = obj.xp(1,i) + obj.Ts * obj.xp(2,i);
                end
            end


            %% Form distribution from the propagated particles
            % Calculate Mean (should be equal to single propagated mean
            xbar = mean(obj.xp,2);

            % Calculate Covariance
            Pap = zeros(3,3);
            for i = 1:obj.numParticles
                Pap = Pap + (obj.xp(:,i) - xbar) * ((obj.xp(:,i) - xbar))';
            end
            Pap = Pap/obj.numParticles;

            %% Calculate 3sigma bound from the covariance matrix (upper and lower limits)
            sigma_particles = sqrt([Pap(1,1), Pap(2,2), Pap(3,3)]); %find SD, only need diagonal elemtns

            apogeeSigma = sigma_particles(1);
            apogeePrediction = xbar(1);

            % Update Estimate of Apogee
            obj.apogeePred = apogeePrediction;
            obj.sigmaApogee = apogeeSigma;

            % Add to history of apogee estimates
            obj.apogeePredHist = [obj.apogeePredHist apogeePrediction];
            obj.sigmaApogeeHist = [obj.sigmaApogeeHist apogeeSigma];
        end
    end
end

