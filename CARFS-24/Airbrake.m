classdef Airbrake
    properties
        position
        velocity
        maxPosition
        minPosition
        maxVelocity
        minVelocity
        Cd = 0.5  % Drag coefficient
        A = 0.03   % Reference area
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
                end
            end
        end
        
        % Method to calculate Cd * A based on the position
        function cd_times_A = getCdTimesA(obj, machNumber)
            % Assuming a linear relationship between position and Cd
            cd_at_position = obj.Cd * (obj.position / obj.maxPosition);
            
            % Calculate Cd * A
            cd_times_A = cd_at_position * obj.A;
        end
    end
end
