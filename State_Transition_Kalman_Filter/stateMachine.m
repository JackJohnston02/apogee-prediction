classdef stateMachine
    %STATEMACHINE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        pad
        burning
        coasting
        descent
        landed

        padToBurning 
        burningToCoasting
        coastingTodescent
        descentToLanded 

        apogeeAltitude
    end
    
    methods
        function obj = stateMachine()
            %STATEMACHINE Construct an instance of this class
            %   Detailed explanation goes here
            obj.pad = true;
            obj.burning = false;
            obj.coasting = false;
            obj.descent = false;
            obj.landed = false;
        end
        
        function obj = check(obj, x, t)

            %Check for transition from pad to burning ~ launch
            if obj.pad == true && x(2) > 5
                obj.pad = false;
                obj.burning = true;
                obj.padToBurning = t;
            end
    
            % Check for transition from burning to coasting ~ burnout
            if obj.burning == true && t > obj.padToBurning + 3
                obj.burning = false;
                obj.coasting = true;
                obj.burningToCoasting = t;
            end
            
            % Check for transition from coasting to descent ~ apogee
            if obj.coasting == true && abs(x(2)) < 1;
                obj.coasting = false;
                obj.descent = true;
                obj.coastingTodescent = t;
                obj.apogeeAltitude = x(1);
            end

            % Check for transition from descent to landed ~ touchdown
            if obj.descent == true && abs(x(2)) < 1 && x(1) < obj.apogeeAltitude - 200
                obj.descent = false;
                obj.landed = true;
                obj.descentToLanded = t;
            end
        end
        


    end
end
