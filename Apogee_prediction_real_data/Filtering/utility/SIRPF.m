classdef SIRPF
    %SIRPF Sequential Importance Resampling Particle Filter (Bootstrap)
    %   Detailed explanation goes here

    properties
        xpk_1
        xpk
        wpk_1
        wpk
        f
        h
        Q
        R
        N
        n
        m
        xhat
        resampling_percentage
        resampling_strategy
        Nthresh
    end

    methods (Static)
        function obj = SIRPF(xi,Pi,f,h,N,Q,R,resamp_perc,resamp_strat)
            %SIRPF Construct an instance of the SIR-PF
            % System and Observation Models
            obj.f = f;
            obj.h = h;
            obj.N = N;
            obj.Q = Q;
            obj.R = R;
            % State and measurement dimensions
            obj.n = size(Pi,1);
            obj.m = size(R,1);

            % Initialise Weights and Particles
            obj.wpk_1 = repmat(1/obj.N,obj.N,1);
            obj.wpk = repmat(1/obj.N,obj.N,1);
            obj.xpk_1 = zeros(obj.n,obj.N);
            obj.xpk = zeros(obj.n,obj.N);
            for i = 1:obj.N
                obj.xpk_1(:,i) = mvnrnd(xi,Pi);
            end

            obj.resampling_percentage = resamp_perc;
            obj.resampling_strategy = resamp_strat;
            obj.Nthresh = obj.resampling_percentage*obj.N;
        end

        function obj = recursion(obj,yk)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here


            for i = 1:obj.N
                % Draw sample using propagated particle as the mean using
                % the process noise to expand the uncertainty
                obj.xpk(:,i) = mvnrnd(obj.f(obj.xpk_1(:,i)),obj.Q);
                % Update weight using the a prior PDF
                % obj.wpk(i) = obj.wpk_1(i)*mvnpdf((yk - obj.h(obj.xpk(:,i))),zeros(obj.m,1),obj.R);
                obj.wpk(i) = mvnpdf((yk - obj.h(obj.xpk(:,i))),zeros(obj.m,1),obj.R);
            end

            % Normalise weights
            obj.wpk = obj.wpk./sum(obj.wpk);

            %% Resample
            obj = obj.resample(obj);

            % Compute state estimate
            obj.xhat = obj.xpk*obj.wpk;
            % obj.xhat = mean(obj.xpk,2);

            % Update notation
            obj.xpk_1 = obj.xpk;
            obj.wpk_1 = obj.wpk;


            %% END OF FUNCTION
        end

        function obj = resample(obj)
            % Extract number of particles
            N = length(obj.wpk);

            switch obj.resampling_strategy
                case 'multinomial'
                    idx = randsample(1:N,N,'true',obj.wpk);
                case 'systematic'
                    edges = min([0 cumsum(obj.wpk)'],1);
                    u1 = rand/N;
                    edges(end) = 1;
                    [~,idx] = histc(u1:1/N:1,edges);
                case 'random_uniform'
                    idx = find(rand <= cumsum(obj.wpk),1);
                case 'stratified'
                    [m,n] = size(obj.wpk);
                    mn = m.*n;
                    pn = obj.wpk./sum(obj.wpk(:)).*mn;
                    s = zeros(m,n);
                    r = rand(1,mn);
                    k = 0;
                    c = 0;
                    for i = 1:mn
                        c = c + pn(i);
                        if c >= 1
                            a = floor(c);
                            c = c - a;
                            idx(k+[1:a]) = i;
                            k = k + a;
                        end
                        if k < mn && c >= r(k+1)
                            c = c - 1;
                            k = k + 1;
                            idx(k) = i;
                        end
                    end

                otherwise
                    errordlg('\nResampling strategy not selected.');
            end

            % Resample particles
            obj.xpk = obj.xpk(:,idx);

            % Re-weight particles
            N = size(obj.xpk,2);
            obj.wpk = repmat(1/N,N,1);

            % Count Particles
            obj.N = N;
            

            %% END OF FUNCTION
        end
    end
end

