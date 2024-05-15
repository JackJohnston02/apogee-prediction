classdef Airbrake
    properties
        position
        maxPosition
        minPosition
        Cd = 1.8  % Drag coefficient, function of Mach number
        A = (57 * 10^(-3) * 10*10^(-2))   % Reference area, at max deployment
    end
    
    methods
        % Constructor function
        function obj = Airbrake(maxPos, minPos)
            obj.maxPosition = maxPos;
            obj.minPosition = minPos;
            obj.position = minPos;  % Initial position
        end
        
        
        % Method to update the position of the airbrake based on the velocity
        function obj = updatePosition(newPosition)
          obj.position = newPosition;
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
