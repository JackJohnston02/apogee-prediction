function [thrust] = motor_thrust(t,motor_path)

% Create the file path
file_path = fullfile(motor_path);

% Open the file
fileID = fopen(file_path, 'r');

% Check if file exists
if fileID == -1
    error('File not found');
end

% Read the file
thrust_data = textscan(fileID, '%f %f', 'HeaderLines', 1);

% Close the file
fclose(fileID);

% Check if data is empty
if isempty(thrust_data{1}) || isempty(thrust_data{2})
    error('No data in file');
end

% Convert cell array to matrix
thrust_data = cell2mat(thrust_data);

% Interpolate the thrust at time t
thrust = interp1(thrust_data(:,1), thrust_data(:,2), t, 'linear', 'extrap');

thrust(thrust < 0) = 0;
end
