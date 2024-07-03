% Load data
clear all
filename = 'Regulus 100.0Hz.csv';
data = readtable(filename);
column_headers = data.Properties.VariableNames;
data_struct = struct();

% Store data in a struct
for i = 1:length(column_headers)
    column_name = column_headers{i};
    column_data = data.(column_name);
    data_struct.(column_name) = column_data;
end

disp(column_headers)
normLat = (data_struct.Latitude - data_struct.Latitude(1));
normLon = (data_struct.Longitude - data_struct.Longitude(1));

normLat = (normLat/(max(abs(normLat))));
normLon = (normLon/max(abs(normLon)));

normX = data_struct.X / max(abs(data_struct.X));
normY = data_struct.Y / max(abs(data_struct.Y));
normZ = data_struct.Z / max(abs(data_struct.Z));

hold on
scatter3(normLon,normLat, data_struct.Z)
scatter3(normX, normY, data_struct.Z)
hold off
xlabel('Longitude');
ylabel('Latitude');
zlabel('Altitude');
hold off;


%% Attitude Visualization

%% Attitude Visualization

% Assuming data_struct contains Euler parameters in ENU format
e0 = data_struct.Euler0;
e1 = data_struct.Euler1;
e2 = data_struct.Euler2;
e3 = data_struct.Euler3;

% Convert Euler parameters to quaternion in NED format
q_ned = [e0; -e1; -e2; -e3];

% Assuming e3_ENU and e3_NED are your reference vectors
% Rotate reference vectors using quaternions
num_vectors = size(e3_ENU, 1);  % Assuming e3_ENU and e3_NED have the same number of rows
sim.s3 = zeros(num_vectors, 3);
sim.s3New = zeros(num_vectors, 3);

for i = 1:num_vectors
    % Rotate using ENU quaternion
    sim.s3(i,:) = quatrotate(q_ned', e3_ENU(i,:));  % Note the transpose of q_ned for compatibility with quatrotate
    
    % Rotate using NED quaternion
    sim.s3New(i,:) = quatrotate(q_ned', e3_NED(i,:)); % Note the transpose of q_ned for compatibility with quatrotate
end

% Create 3D plot
figure;
grid minor;
hold on;

% Plot a sphere for reference
[x,y,z] = sphere(30);
surf(x,y,z,'FaceAlpha',0.1,'DisplayName','Attitude Sphere');

xlabel('X NED [m]');
ylabel('Y NED [m]');
zlabel('Z NED [m]');

% Plot attitude vectors in ENU and NED frames
plot3(sim.s3(:,1), sim.s3(:,2), sim.s3(:,3), 'Color', 'b', 'DisplayName', 'Attitude in ENU Frame', 'LineWidth', 2);
plot3(sim.s3New(:,1), sim.s3New(:,2), sim.s3New(:,3), 'Color', 'g', 'DisplayName', 'Attitude in NED Frame', 'LineWidth', 2);

hold off;
legend;



