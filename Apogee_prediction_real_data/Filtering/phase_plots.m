clear all;
close all;

% Set default text interpreter to LaTeX for better formatting in plots
set(groot, 'defaulttextinterpreter', 'latex');
set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');

% General settings for plots
markerSize = 1;  % Symbol size

% Define custom colors
greyColor = [0.5, 0.5, 0.5]; % Grey color

% File settings
folder = 'data_filtered';
csvFiles = dir(fullfile(folder, '*.csv'));
fileNames = {csvFiles.name};

% Display available files for user selection
for i = 1:length(fileNames)
    disp(string(i) + " : " + fileNames(i));
end

% Prompt user to select a file
file_index = input("Choose File (enter the corresponding number): ");
while file_index < 1 || file_index > length(fileNames)
    disp("Invalid choice. Please enter a valid number.");
    file_index = input("Choose File (enter the corresponding number): ");
end

% Read the selected CSV file
file_choice = fileNames{file_index};
file_path = fullfile(folder, file_choice);
data = csvread(file_path, 1, 0);

% Extract columns (assumes specific column order)
time = data(:, 1);        % Time
altitude = data(:, 2);       % Altitude
velocity = data(:, 3);       % Velocity
acceleration = data(:,4);
ballistic_coefficient = data(:, 5);

% Downsample the data (adjust the factor as needed)
downsample_factor = 1; 
indices = 1:downsample_factor:length(time);
time = time(indices);  % Downsampled Time
altitude = altitude(indices);
velocity = velocity(indices);
acceleration = acceleration(indices);
ballistic_coefficient = ballistic_coefficient(indices);

%% Generate the constant Cb curves
mean_ballistic_coefficient = mean(ballistic_coefficient);
apogee = max(altitude);
max_velocity = max(velocity);

n = 5; % Number of isolines
m = 10000;

Cb_lb = round((mean_ballistic_coefficient - mean_ballistic_coefficient/2)/50)*50;
Cb_ub = round((mean_ballistic_coefficient + mean_ballistic_coefficient/2)/50)*50;

% Define the lower bound with a minimum threshold
Cb_lb = max(Cb_lb, 100);

% Generate values
Cb_inputs = linspace(Cb_lb, Cb_ub, n);

% Ensure each value is a multiple of 50
multiple = 50;
Cb_inputs = round(Cb_inputs / multiple) * multiple;

altitude_out = NaN(n, m);
velocity_out = NaN(n, m);
Cb_out = NaN(n, m);

dt = 0.01;

for i = 1:length(Cb_inputs)
    Cb = Cb_inputs(i);
    x(1) = apogee;
    x(2) = 0;
    x(3) = get_gravity(x(1));
    x(4) = Cb;
    j = 0;
    
    while x(1) > 0 && x(2) < max_velocity && j < length(altitude_out(i,:))
        j = j + 1;
        altitude_out(i,j) = x(1);
        velocity_out(i,j) = x(2);
        Cb_out(i,j) = x(4);

        x = processModel_backward(x, dt);
    end
end

% Plot the results
figure;
hold on;
xlim([0, max_velocity + max_velocity/10]);
ylim([0, apogee + apogee/10]);

% Plot each curve with dashed grey lines
for i = 1:length(Cb_inputs)
    % Plot with dashed grey lines
    plot(velocity_out(i,:), altitude_out(i,:), 'Color', greyColor, 'LineStyle', '--', 'DisplayName', ['Cb = ' num2str(Cb_inputs(i))]);

    % Add text labels inline with the data
    % Find a suitable position along the curve for each label
    % Choose a point for the label based on curve length or density
    [maxValue, label_index] = max(velocity_out(i, :));
    label_index = label_index;

    text_pos = [velocity_out(i, label_index), altitude_out(i, label_index)-10];
    angle = -45;
    
    % Add the label with rotation
    text(text_pos(1), text_pos(2), ['Cb = ' num2str(Cb_inputs(i))], 'Color', greyColor, 'FontSize', 8, ...
        'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left', 'Rotation', angle);
end




% Add legend

% Need to plot motor burning and non burning as two seperate colours
 [maxValue, index_motor_burnout] = max(velocity);

% Plot first segment with a custom color
pre_burnout = plot(velocity(1:index_motor_burnout), altitude(1:index_motor_burnout), ...
     'LineWidth', 2, 'Color', "#fb6025");  % Light blue color

% Plot second segment with another custom color
post_burnout = plot(velocity(index_motor_burnout:end), altitude(index_motor_burnout:end), ...
     'LineWidth', 2, 'Color', "#45ae8d");  % Orange color

% Add legend with only specific lines
legend([pre_burnout, post_burnout], {'Pre-Burnout', 'Post-Burnout'});

hold off;
xlabel('Velocity');
ylabel('Altitude');
title('Altitude vs. Velocity');

saveas(gcf, 'plots/Phase_plot_' + string(fileNames(file_index)) + '.png');

% Supporting functions
function x_new = processModel_backward(x_s, dt)
    dt = -dt;
    g = get_gravity(x_s(1));
    rho = get_density(x_s(1));

    % Update altitude (decreasing since we're propagating backwards)
    x_s(1) = x_s(1) + x_s(2) * dt + 0.5 * x_s(3) * dt^2;
    
    % Update velocity (account for drag and gravity)
    x_s(2) = x_s(2) + x_s(3) * dt;
    
    % Update acceleration (gravity and drag)
    x_s(3) = g - (rho * x_s(2)^2) / (2 * x_s(4));
    
    % Ballistic coefficient remains the same
    x_s(4) = x_s(4);

    x_new = [x_s(1); x_s(2); x_s(3); x_s(4)];
end

function rho = get_density(h)
% Returns atmospheric density as a function of altitude
% Accurate up to 11km
% https://en.wikipedia.org/wiki/Density_of_air

p_0 = 101325; % Standard sea level atmospheric pressure
M = 0.0289652; % molar mass of dry air
R = 8.31445; % ideal gas constant
T_0 = 288.15; % Standard sea level temperature
L = 0.0065; % temperature lapse rate
g = get_gravity(h);

rho = (p_0 * M)/(R * T_0) * (1 - (L * h)/(T_0))^(((-g * M) / (R* L)) - 1); % -g used as g is -ve by default
end

function g = get_gravity(h)
% Returns gravity as a function of altitude
% Approximates the Earth's gravity assumes a perfect sphere

g_0 = -9.80665; % Standard gravity
R_e = 6371000; % Earth radius

g = g_0 * (R_e / (R_e + h))^2;
end
