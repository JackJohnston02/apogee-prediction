classdef Airbrake
    properties
        % Define the properties of the airbrake here
        % For example:
        position
        maxPosition
        minPosition
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
        
        % Add more methods as needed for your simulation
    end
end
