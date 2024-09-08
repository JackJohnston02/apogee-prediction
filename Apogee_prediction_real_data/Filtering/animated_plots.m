clear all;
close all;

% Settings
aspect_ratio = [600, 1000]; % Aspect ratio (width, height)
figure_scale = 1000; % Scale factor for the figure size

% Calculate figure dimensions based on aspect ratio and scale
figure_width = aspect_ratio(1) * figure_scale / aspect_ratio(2);
figure_height = figure_scale;

folder = 'data_filtered';

% Get a list of all CSV files in the folder
csvFiles = dir(fullfile(folder, '*.csv'));

% Extract the filenames from the structure array
fileNames = {csvFiles.name};

% Display the list of files to the user
for i = 1:length(fileNames)
    disp(string(i) + " : " + fileNames(i));
end

% Prompt the user to select a file by entering a number
file_index = input("Choose File (enter the corresponding number): ");

% Ensure the user selects a valid file index
while file_index < 1 || file_index > length(fileNames)
    disp("Invalid choice. Please enter a valid number.");
    file_index = input("Choose File (enter the corresponding number): ");
end

% Get the selected file name and full path
file_choice = fileNames{file_index};
file_path = fullfile(folder, file_choice);

% Read the data from the selected CSV file (assuming the data starts from the second row and all columns are relevant)
data = csvread(file_path, 1, 0);

% Extract time and state columns from the data
times = data(:, 1);    % Time is the first column
state1 = data(:, 2);   % State 1 (Altitude) is the second column
state2 = data(:, 3);   % State 2 (Velocity) is the third column
state3 = data(:, 4);   % State 3 (Acceleration) is the fourth column
state4 = data(:, 5);   % State 4 (Ballistic Coefficient) is the fifth column
predicted_apogee = data(:, 6); % New State 5 (Predicted Apogee) is the sixth column

% Downsample the data
downsample_factor = 10; % Adjust this factor as needed
indices = 1:downsample_factor:length(times);

times_ds = times(indices);
state1_ds = state1(indices);
state2_ds = state2(indices);
state3_ds = state3(indices);
state4_ds = state4(indices);
predicted_apogee_ds = predicted_apogee(indices);

% Calculate individual y-limits for each state with a 10% margin
range1 = max(state1_ds) - min(state1_ds);
range2 = max(state2_ds) - min(state2_ds);
range3 = max(state3_ds) - min(state3_ds);
range4 = max(state4_ds) - min(state4_ds);

ylim1 = [min(state1_ds), max(state1_ds) + 0.2 * range1];
ylim2 = [min(state2_ds), max(state2_ds) + 0.2 * range2];
ylim3 = [min(state3_ds) - 0.1 * range3, max(state3_ds) + 0.1 * range3];
ylim4 = [min(state4_ds) - 0.1 * range4, max(state4_ds) + 0.1 * range4];

% Calculate frame rate
time_intervals = diff(times_ds);
avg_time_interval = mean(time_intervals); % Average time interval between samples
frame_rate = 1 / avg_time_interval; % Frames per second

% Set default text interpreter to LaTeX for better formatting in plots
set(groot, 'defaulttextinterpreter', 'latex');
set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');

% Define colors for the plots
color1 = "#627db8"; % Default MATLAB blue
color2 = "#627db8"; % Default MATLAB orange
color3 = "#627db8"; % Default MATLAB yellow
color4 = "#627db8"; % Default MATLAB purple
color5 = "#45ae8d"; % Default MATLAB green

% Setup for animated subplots
figure('Position', [100, 100, figure_width, figure_height]);

sgtitle('UKF State Estimation', 'Interpreter', 'latex', 'FontWeight', 'bold');


% Create the 4x1 subplots
subplot(4, 1, 1);
hold on;
plot_handle1 = plot(NaN, NaN, 'LineWidth', 1.5, 'Color', color1, 'DisplayName', 'Altitude');
scatter_handle1 = scatter(NaN, NaN, 20, 'x', 'MarkerEdgeColor', color5, 'DisplayName', 'Predicted Apogee');
ylabel('Altitude ($m$)', 'Interpreter', 'latex', 'FontWeight', 'bold');
legend('show', 'Location', 'southeast', 'FontWeight', 'bold'); % Set legend location to southeast
grid on;
hold off;

subplot(4, 1, 2);
plot_handle2 = plot(NaN, NaN, 'LineWidth', 1.5, 'Color', color2);
ylabel('Velocity ($m/s$)', 'Interpreter', 'latex', 'FontWeight', 'bold');
grid on;

subplot(4, 1, 3);
plot_handle3 = plot(NaN, NaN, 'LineWidth', 1.5, 'Color', color3);
ylabel('Acceleration ($m/s^2$)', 'Interpreter', 'latex', 'FontWeight', 'bold');
grid on;

subplot(4, 1, 4);
plot_handle4 = plot(NaN, NaN, 'LineWidth', 1.5, 'Color', color4);
xlabel('Time (s)', 'Interpreter', 'latex', 'FontWeight', 'bold');
ylabel('Ballistic Coefficient $(kg/m^2)$', 'Interpreter', 'latex', 'FontWeight', 'bold');
grid on;

% Create a VideoWriter object to save the animation
video_filename = 'animated_plots/new_animated_plot.mp4';
video = VideoWriter(video_filename, 'MPEG-4');
video.FrameRate = frame_rate; % Set the frame rate to match the data interval
open(video);

% Animation loop
start_time = tic; % Start a timer to keep track of elapsed real time

for t = 1:length(times_ds)
    % Update subplot 1 (Altitude and Predicted Apogee)
    subplot(4, 1, 1);
    set(plot_handle1, 'XData', times_ds(1:t), 'YData', state1_ds(1:t));
    
    % Check condition for predicted apogee
    condition = state3_ds(1:t) < -9.81;
    scatter_x = times_ds(1:t);
    scatter_y = predicted_apogee_ds(1:t);
    
    % Only plot scatter points if the condition is met
    if any(condition)
        % Scatter points where condition is met
        set(scatter_handle1, 'XData', scatter_x(condition), 'YData', scatter_y(condition));
    else
        % Clear scatter points if condition is not met
        set(scatter_handle1, 'XData', NaN, 'YData', NaN);
    end
    
    xlim([min(times_ds) max(times_ds)]);
    ylim(ylim1);
    
    % Set y-axis ticks with increments of 500
    ytick_interval = 1000;
    y_min = floor(ylim1(1) / ytick_interval) * ytick_interval;
    y_max = ceil(ylim1(2) / ytick_interval) * ytick_interval;
    set(gca, 'YTick', y_min:ytick_interval:y_max);
    
    % Update subplot 2 (Velocity)
    subplot(4, 1, 2);
    set(plot_handle2, 'XData', times_ds(1:t), 'YData', state2_ds(1:t));
    xlim([min(times_ds) max(times_ds)]);
    ylim(ylim2);

    % Update subplot 3 (Acceleration)
    subplot(4, 1, 3);
    set(plot_handle3, 'XData', times_ds(1:t), 'YData', state3_ds(1:t));
    xlim([min(times_ds) max(times_ds)]);
    ylim(ylim3);

    % Update subplot 4 (Ballistic Coefficient)
    subplot(4, 1, 4);
    set(plot_handle4, 'XData', times_ds(1:t), 'YData', state4_ds(1:t));
    xlim([min(times_ds) max(times_ds)]);
    ylim(ylim4);

    % Capture the current frame and write it to the video file
    frame = getframe(gcf);
    writeVideo(video, frame);

    % Calculate the real-time pause duration based on time difference
    if t > 1
        elapsed_time = toc(start_time); % Get the elapsed real time
        time_diff = times_ds(t) - times_ds(t-1); % Calculate the time difference in the data
        pause_duration = time_diff - elapsed_time; % Calculate the pause duration
        if pause_duration > 0 % Only pause if there's time left
            pause(pause_duration);
        end
        start_time = tic; % Reset the timer for the next iteration
    end
end

% Close the video file
close(video);

disp(sprintf('Animation complete and saved to %s', video_filename));
close all;
