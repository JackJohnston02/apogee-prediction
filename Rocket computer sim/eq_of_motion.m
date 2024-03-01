classdef eq_of_motion
    %EQ_OF_MOTION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        acceleration
        angular_momentum
    end
    
    methods
        function obj = eq_of_motion(v,t)
            %EQ_OF_MOTION Construct an instance of this class
            %   Detailed explanation goes here
            obj.acceleration =  9.81 -v.*v;
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end

