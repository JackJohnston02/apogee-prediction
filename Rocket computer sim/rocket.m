classdef rocket
    %class containing the physical properties of the rocket
    %   Detailed explanation goes here

    properties
        mass {mustBeInteger, mustBePositive}
        diameter {mustBeInteger, mustBePositive}
        length {mustBeInteger, mustBePositive}
        area {mustBeInteger, mustBePositive}
        inertia 
        cd {mustBePositive}
    end

    methods
        function obj = untitled(inputArg1,inputArg2)
            %UNTITLED Construct an instance of this class
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