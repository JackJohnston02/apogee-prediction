classdef PIDController
    properties
        Kp
        Ki
        Kd
        setpoint
        integralTerm
        lastError
    end
    
    methods
        function obj = PIDController()
            obj.Kp = 3;
            obj.Ki = 0;
            obj.Kd = 0;
            obj.integralTerm = 0;
            obj.lastError = 0;
            obj.setpoint = 0;
        end
        
       function output = calculate(obj, setpoint, process_variable)
            error = process_variable - setpoint;
            P = obj.Kp * error;
            obj.integralTerm = obj.integralTerm + obj.Ki * error;
            derivative = obj.Kd * (error - obj.lastError);
            obj.lastError = error; 
            output = P + obj.integralTerm + derivative;
        end
    end
end
