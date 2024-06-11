classdef Airbrake
%Todo
    %Method for updating maximum motor speed based on experienced drag
    %force Fd

    %Not convinced I havent made a mistake somewhere, the output plots seem
    %an awful lot faster than expected?

    properties
        %Flap properties
        maxAngle
        minAngle
        angle
        Cd
        A
        Fd

        %Motor properties
        maxVelocity
        minVelocity
        minAccleration
        maxAccleration
        desiredVelocity
        velocity

        %Mechanism properties
        L1
        L2
        maxP
        minP
        P
        pitch
        gearRatio

    end
    
    methods
        % Constructor function
        function obj = Airbrake()
            %Flap properties
            obj.maxAngle = 55;
            obj.minAngle = 0;
            obj.angle = obj.minAngle;  % Initial position
            obj.Cd = 1.8;  % Drag coefficient, function of Mach number
            obj.A = (57 * 10^(-3) * 165*10^(-3));   % Reference area, at 90 degrees  

            %Motor properties - https://www.nanotec.com/eu/en/products/1333-sc4118l1804-eno05k#dimensions
            obj.maxAccleration = 50/60; %RPMPS;
            obj.minAccleration = -obj.maxAccleration;
            obj.maxVelocity = 1000/60;%RPM for motor
            obj.minVelocity = -obj.maxVelocity;%RPS for motor
            obj.desiredVelocity = 0;
            obj.velocity = 0/60;%RPS
            
            %Mechanism properties
            obj.L1 = 45/1000;%m, Length of upper control arm
            obj.L2 = 57/1000;%m, Length of lower control arm
            obj.maxP = sqrt(obj.L1^2 - 2*(obj.L1 * sin(deg2rad(obj.minAngle)))^2 + obj.L2^2);
            obj.minP = sqrt(obj.L1^2 - 2*(obj.L1 * sin(deg2rad(obj.maxAngle)))^2 + obj.L2^2);
            obj.P = sqrt(obj.L1^2 - 2*(obj.L1 * sin(deg2rad(obj.angle)))^2 + obj.L2^2); %mm, Initial crucifix position
            obj.pitch = 6/1000;%mm, Pitch of lead screw threads
            obj.gearRatio = 1;%x:1        
        end
        
        
        
        % Method to calculate Cd * A based on the flap mposition
        function cd_times_A = getCdTimesA(obj, machNumber)
            % Calculate effective area from angle and ref A
            A_effective = obj.A * sin(deg2rad(obj.angle));
            % Calculate Cd * A
            cd_times_A = obj.Cd * A_effective;
        end

        % Method to update the crucifix and airbrake position
        function obj = updateAirbrakes(obj, dt)
            
       

            if (obj.desiredVelocity - obj.velocity)/dt > obj.maxAccleration
                obj.velocity = obj.velocity + dt * obj.maxAccleration;
            

            elseif (obj.desiredVelocity - obj.velocity)/dt < obj.minAccleration
                obj.velocity = obj.velocity + dt * obj.minAccleration;
            
            else
                obj.velocity = obj.desiredVelocity;
            end

            if obj.velocity > obj.maxVelocity
                obj.velocity = obj.maxVelocity; 
    
            elseif obj.velocity < obj.minVelocity
                    obj.velocity = obj.minVelocity; 
            end
            

            %update crucifix position
            linVel = obj.pitch * obj.velocity/obj.gearRatio; %velocity is pitch * RPS
            obj.P = obj.P + linVel * dt;
            %Check P for the limits
            if obj.P > obj.maxP
                %disp("maxxed out P " + obj.P );
                obj.P = obj.maxP;
            end
            if obj.P < obj.minP
                %disp("mined out P " + obj.P);
                obj.P = obj.minP;
            end
            %update flap angle
            obj.angle = rad2deg(asin((1/obj.L1) * (sqrt((obj.L1^2 + obj.L2^2 - obj.P^2)/(2)))));
        end

    end
end
