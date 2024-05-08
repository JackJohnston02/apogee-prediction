classdef Airbrake
    properties
        position
        velocity
        maxPosition
        minPosition
        maxVelocity
        minVelocity
        Cd = 1.8  % Drag coefficient, function of Mach number
        A = (57 * 10^(-3) * 10*10^(-2))   % Reference area, at max deployment
    end
    
    methods
        % Constructor function
        function obj = Airbrake(maxPos, minPos)
            obj.maxPosition = maxPos;
            obj.minPosition = minPos;
            obj.maxVelocity = 10;
            obj.minVelocity = -10;
            obj.position = minPos;  % Initial position
            obj.velocity = 0;  % Initial velocity
        end
        
        % Method to update the velocity of the airbrake
        function obj = updateVelocity(obj, newVel)
            if newVel >= obj.minVelocity && newVel <= obj.maxVelocity
                obj.velocity = newVel;
            else
                %fprintf('New velocity is outside of allowable range. Velocity is saturated.\n');
                if newVel < obj.minVelocity
                    obj.velocity = obj.minVelocity;
                else
                    obj.velocity = obj.maxVelocity;
                end
                
            end
        end
        
        % Method to update the position of the airbrake based on the velocity
        function obj = updatePosition(obj, dt)
            newPos = obj.position + obj.velocity * dt;
            if newPos >= obj.minPosition && newPos <= obj.maxPosition
                obj.position = newPos;
            else
                %fprintf('New position is outside of allowable range. Position is saturated.\n');
                if newPos < obj.minPosition
                    obj.position = obj.minPosition;
                else
                    obj.position = obj.maxPosition;
                    obj.velocity = 0;
                end
            end
        end
        
        % Method to calculate Cd * A based on the position
        function cd_times_A = getCdTimesA(obj, machNumber)
            % Calculate effective area from angle and ref A
            A_effective = obj.A * sin(deg2rad(obj.position));
            % Calculate Cd * A
            cd_times_A = obj.Cd * A_effective;
        end
    end
end
