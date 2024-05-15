classdef StepperMotor
    %STEPPERMOTOR Summary of this class goes here
    %   Detailed explanation goes here
    %TODO
        %Limit stepper speed based on the force from the airbrakes, inverse
        %jacobian

    properties
        position
        velocity
        maxPosition
        minPosition
        maxVelocity
        minVelocity
        maxTorque
        minTorque
    end
    
    methods
        function obj = StepperMotor()
            obj.maxPosition = 70; %mm Maximum distance for crucifix to move up leadscrew
            obj.minPosition = 0; %Home position
            obj.maxVelocity = 2500;%RPM
            obj.minVelocity = -2500;%RPM

            %Unit conversion
            obj.maxPosition = obj.maxPosition/1000; %m
            obj.minPosition = obj.maxPosition/1000; %m
            obj.maxVelocity = (obj.maxVelocity/60); %RPS

            %
            obj.position = minPos;  % Initial position, Home
            obj.velocity = 0;  % Initial velocity       
        end
        
        %Controller command
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
        
        function obj = updatePosition(obj, dt)
            %... function for chaning the crucifix position based on dt and
            %vel
        

            moveAirbrakes();
        end

        function obj = moveAirbrakes()
            %Takes crucifix position and converts to airbrake angle
            obj.position

            Airbrake.updatePosition(newAirbrakePosition);

        end
        
    end
end

