classdef PIDController
    properties
        K
        Kp
        Ki
        Kd
        setpoint
        integralTerm
        lastError
    end
    
    methods
        function obj = PIDController()
            obj.K = 0.000005; %Pre gain
            obj.Kp = 400; %Proportional gain
            obj.Ki = 300; %Integral gain
            obj.Kd = 100; %Derivative gain
            obj.integralTerm = 0;
            obj.lastError = 0;
            obj.setpoint = 0;
        end
        
       function output = calculate(obj, setpoint, process_variable)
            error = setpoint - process_variable;
            error = obj.K * error;
            P = obj.Kp * error;
            obj.integralTerm = obj.integralTerm + obj.Ki * error;
            derivative = obj.Kd * (error - obj.lastError);
            obj.lastError = error; 
            output = P + obj.integralTerm + derivative;
        end
    end
end
