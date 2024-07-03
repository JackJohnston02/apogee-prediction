classdef Serial_streamer < handle
    properties (Access = private)
        serialPort; % Serial port object
        isStreaming; % Flag to indicate whether streaming is active
        portName;    % Name of the COM port
        baudRate;    % Baud rate for serial communication
    end
    
    methods
        % Constructor
        function obj = Serial_streamer(comPort, baudRate, flag)
            obj.portName = comPort;
            obj.baudRate = baudRate;
            obj.isStreaming = flag; % Initialize streaming flag
            
            if flag == true
                % Check if the port is already open
                ports = serialportlist("available");
                if any(strcmp(ports, comPort))
                    disp("Port " + comPort + " is already in use. Closing...");
                    obj.closePort(comPort);
                end
                
                % Create and open the serial port object
                obj.serialPort = serialport(comPort, baudRate);
                fopen(obj.serialPort); % Open serial connection
                
                disp("Serial port " + comPort + " opened successfully.");
            end
        end

        % Close the specified port
        function closePort(obj, portName)
            try
                obj.serialPort = serialport(portName, "OutputBufferSize", 1024);
                fclose(obj.serialPort);
                delete(obj.serialPort);
                disp("Port " + portName + " closed successfully.");
            catch
                warning("Failed to close port " + portName + ".");
            end
        end

        % Start streaming velocity data
        function startStreaming(obj, velocity)
            if obj.isStreaming
                fprintf(obj.serialPort, '%f\n', velocity);
            end
        end
        
        % Destructor
        function delete(obj)
            if obj.isStreaming
                fprintf(obj.serialPort, '%f\n', 0.01);
                fclose(obj.serialPort); % Close serial connection if streaming is active          
            end
            delete(obj.serialPort); % Delete serial port object
        end
    end
end
