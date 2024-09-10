% Assuming filterTimes has been populated during the main loop

% Define filter types (you can use dynamic filter names here if needed)
filterNames = fieldnames(filterTimes);

% Initialize arrays to store average times for each step and filter
numFilters = length(filterNames);

avgTimeUpdates = zeros(1, numFilters);
avgMeasurementUpdates = zeros(1, numFilters);
avgApogeePredictions = zeros(1, numFilters);

% Loop through each filter type to calculate averages
for i = 1:numFilters
    filter_type = filterNames{i};  % Get the filter type name
    
    % Calculate the averages for each step
    avgTimeUpdates(i) = mean(filterTimes.(filter_type).timeUpdateTimes);
    avgMeasurementUpdates(i) = mean(filterTimes.(filter_type).measurementUpdateTimes);
    avgApogeePredictions(i) = mean(filterTimes.(filter_type).apogeePredictionTimes);
end

% ---- Plot 1: Time Update ----
figure;
bar(avgTimeUpdates);
set(gca, 'XTick', 1:numFilters, 'XTickLabel', filterNames);  % Set the X-axis ticks and labels
xlabel('Filter Types');
ylabel('Average Time (seconds)');
title('Average Time Update Times for Each Filter');
xtickangle(45);  % Rotate x-axis labels for readability
grid on;

% ---- Plot 2: Measurement Update ----
figure;
bar(avgMeasurementUpdates);
set(gca, 'XTick', 1:numFilters, 'XTickLabel', filterNames);  % Set the X-axis ticks and labels
xlabel('Filter Types');
ylabel('Average Time (seconds)');
title('Average Measurement Update Times for Each Filter');
xtickangle(45);  % Rotate x-axis labels for readability
grid on;

% ---- Plot 3: Apogee Prediction ----
figure;
bar(avgApogeePredictions);
set(gca, 'XTick', 1:numFilters, 'XTickLabel', filterNames);  % Set the X-axis ticks and labels
xlabel('Filter Types');
ylabel('Average Time (seconds)');
title('Average Apogee Prediction Times for Each Filter');
xtickangle(45);  % Rotate x-axis labels for readability
grid on;
