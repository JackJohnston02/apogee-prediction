classdef relative_motion
    %state vector
    %   contains postion xyz then velocity xyz and acceleration xyz from on
    %   axis to another 

    properties
        Property1
    end

    methods
        function obj = calc(axis_1,axix_2)
            %UNTITLED3 Construct an instance of this class
            %   Detailed explanation goes here
            obj.Property1 = inputArg1 + inputArg2;
        end

        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end