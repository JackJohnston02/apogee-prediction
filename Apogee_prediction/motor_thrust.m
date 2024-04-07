function [thrust_derivative] = motor_thrust(t,motor_name)

% Create the file path
file_path = fullfile('motors/' + motor_name +  '.eng');

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

% Compute the derivative of the thrust
time_diff = diff(thrust_data(:,1));
force_diff = diff(thrust_data(:,2));
thrust_derivative = force_diff ./ time_diff;

% Set any negative values to zero
thrust_derivative(thrust_derivative < 0) = 0;

% Interpolate the derivative data
thrust_derivative = interp1(thrust_data(1:end-1,1), thrust_derivative, t, 'linear', 'extrap');


end
