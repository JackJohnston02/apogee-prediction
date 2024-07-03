% Load dat
filename = 'data/real/owen.csv';
data = readtable(filename);
data = data(1:400, :);
column_headers = data.Properties.VariableNames;
data_struct = struct();

% Store data in a struct
for i = 1:length(column_headers)
    column_name = column_headers{i};
    column_data = data.(column_name);
    data_struct.(column_name) = column_data;
end
data_struct.times = (data_struct.timestamp - data_struct.timestamp(1))/1000;
disp("Barometer STD: " + std(data_struct.baro_altitude))
disp("IMUZ STD: " + std(data_struct.imu_accZ))

plot(data_struct.times, data_struct.baro_altitude)