classdef Airbrake
    properties
        position
        maxPosition
        minPosition
        Cd  % Drag coefficient
        A   % Reference area
    end
    
    methods
        % Constructor function
        function obj = Airbrake(maxPos, minPos)
            obj.maxPosition = maxPos;
            obj.minPosition = minPos;
            obj.position = minPos;  % Initial position
        end
        
        % Method to update the position of the airbrake
        function obj = updatePosition(obj, newPos)
            if newPos >= obj.minPosition && newPos <= obj.maxPosition
                obj.position = newPos;
            else
                error('New position is outside of allowable range.')
            end
        end
        
        % Method to calculate Cd * A based on the position
        function cd_times_A = getCdTimesA(obj)
            % Assuming a linear relationship between position and Cd
            cd_at_position = obj.Cd * (obj.position / obj.maxPosition);
            
            % Calculate Cd * A
            cd_times_A = cd_at_position * obj.A;
        end
    end
end
