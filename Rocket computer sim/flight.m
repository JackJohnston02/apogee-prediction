classdef flight
    %FLIGHT Summary of this class goes here
    %   Detailed explanation goes here
   
    properties
        displacement
        velocity
        acceleration
        denisty
        speed_of_sound
    end
    
    methods
        function obj = Flight(inputArg1,inputArg2) 
       
            %FLIGHT Construct an instance of this class
            %   Detailed explanation goes here
            obj.atlitude = inputArg1 + inputArg2;
        end
        
        function outputArg =   (obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end

