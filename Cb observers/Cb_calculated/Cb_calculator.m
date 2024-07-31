clear all;

file = 'kalman_filtered_data.csv';
data = readmatrix(file)';

times = data(:, 1);
altitudes = data(:, 2);
velocities = data(:, 3);
accelerations = data(:, 4);


cbs = zeros(length(times),1);

i = 0;

for i = 1:length(times)

    altitude = altitudes(i);
    velocity = velocities(i);
    acceleration = accelerations(i);
    rho = get_density(altitude);
    g = get_gravity(altitude);

  
    cb = (rho * velocity^2)/(2 * (abs(acceleration) + g));


    cbs(i) = cb;

end

% Save the Cb data for plotting
exportMatrix = [times, cbs];
writematrix(exportMatrix',"owen_calculated.csv") 

figure;
hold on;
plot(times, cbs)
xlim([9, 17]);

hold off

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