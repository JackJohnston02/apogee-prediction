clear all
data = readtable('data/simulated/Regulus 100.0Hz.csv');


timestamp = data.Time;
alt = data.Z;
acc = data.Az;


%% Set up KF
x = [alt(1); 0; 0]; % [altitude; velocity; acceleration]

% Estimation error covariance
P = eye(3);

% System dynamics
H = [1 0 0; 0 0 1]; % Observation model

Q = [1 0 0 ; 
     0 1 0; 
     0 0 1]; % Static rocess noise covariance %0.001 * eye(3) 

R = 1*diag([1, 1]); % Measurement noise covariance

rocket_mass = 9.462;

x_est = [];
x_est(:, 1) = x;
B = [0 0 1]';
u = 0;
uout = [];


%% Set up statemachine

launch_detected = false;
motor_burntout = false;
apogee_detected = false;
landed = false;


launch_time = 0;
burnout_time = 0;
apogee_time = 0;
apogee_altitude = 0;

predicted_apogee_time = 0;
predicted_apogee_altitude = 0;
predicted_apogee_times = [0];
predicted_apogee_altitudes = [0];


lower_bound = 0;
upper_bound = 0;
lower_bounds = [0];
upper_bounds = [0];

prediction_times = [0];


k = 1;
t = 0;
times = [t];
dt = 0.01;


%% Loop

while ~landed && k < length(timestamp)
    t = t + dt;
    times  = [times, t];
    

    Q = 0.002*[(dt^5)/20, (dt^4)/8, (dt^3)/6; (dt^4)/8, (dt^3)/3, (dt^2)/2; (dt^3)/6, (dt^2)/2, dt];%Dynamic process noise covariance
    A = [1 dt 0.5*dt^2; 
        0 1 dt; 
        0 0 1]; % State transition
    
    B = [0 0 dt]';
    %Control Input u ~ only thrust curve for these sims
        %Can't just add the thrust needs to be converted to a force
        %If keep adding force it spirals out of control, force as faction
        %of timestep sorts this.

    if  launch_detected && ~motor_burntout
        u = ((motor_thrust(t, "Cesaroni_4025L1355-P")/rocket_mass) - u);    
    end

    if motor_burntout
        u = -9.81;
    end
    uout = [uout, u];

    % Predict state and estimation error covariance
    x = A*x + B*u;
    P = A*P*A' + Q;

    % Compute Kalman gain
    K = P*H' / (H*P*H' + R);

    %Only update measurement step if new data
    if t >= timestamp(k)
        z = [alt(k); acc(k)]; % Measurement vector
        x = x + K * (z - H*x);
        P = (eye(3) - K*H) * P;
        k = k + 1;
    end
 
    % record the states
    x_est(:, end + 1) = x;



    %% Update state machine
    % Check for launch 
    if ~launch_detected && x_est(3, end) > 1 % Assuming launch when velocity > 10
        launch_detected = true;
        launch_time = t;
        disp("launched");
    end
    
    % Check motor burnout
    if launch_detected && ~motor_burntout && x_est(3, end) < 0 % Assuming motor burnout when acceleration < 0
        motor_burntout = true;
        burnout_time = t;
        disp("burnout");
    end

    % Predict apogee after 0.1 second after motor burn-out and before apogee is detected
    if motor_burntout && t >  0.1 + burnout_time && ~apogee_detected
    
        [lower_bound, predicted_apogee_altitude, upper_bound] =  FP_Model(x, P, t, dt);
        predicted_apogee_altitudes = [predicted_apogee_altitudes, predicted_apogee_altitude];
        lower_bounds = [lower_bounds, lower_bound];
        upper_bounds = [upper_bounds, upper_bound];

        prediction_times = [prediction_times, t];
    end

    % Check apogee
    if motor_burntout && ~apogee_detected && x_est(2, end) < 0
        apogee_detected = true;
        apogee_time = t;
        apogee_altitude = x_est(1, end);
        disp("apogee");
    end

    % Check landed
    if apogee_detected && ~landed && t >  1 + apogee_time && abs(x_est(2, end)) < 1 && abs(x_est(3, end)) < 1
        landed = true;
        landed_time = t;
        landed_altitude = x_est(1, end);
        disp("landed");
    end




end

%% Plot data
% Final apogee detection information
disp(['Apogee detected at time: ', num2str(apogee_time), ' seconds']);
disp(['Apogee altitude: ', num2str(apogee_altitude), ' meters']);


clf;  % Clear  figure
tiledlayout(2,2);

% Altitude estimates and measured data
nexttile;
plot(timestamp, alt, 'y');
hold on;
plot(times, x_est(1, :), 'b');

%Plot lower, mean and upper of apogee predictions

scatter(prediction_times, lower_bounds,2, 'r'); 
scatter(prediction_times, predicted_apogee_altitudes,3, 'g'); 
scatter(prediction_times, upper_bounds,2, 'b'); 

title('Altitude Estimates and Measured Data');
xlabel('Time (s)');
ylabel('Altitude (m)');
legend('Measured Altitude', 'Estimated Altitude', 'Propagated Altitude');



xline(launch_time, 'g:', 'DisplayName', 'Launch Time');

xline(burnout_time, 'g:', 'DisplayName', 'Motor Burnout Time');

xline(apogee_time, 'g', 'DisplayName', 'Actual Apogee Time');
yline(apogee_altitude, 'g', 'DisplayName', 'Actual Apogee Altitude');


hold off;

nexttile;

scatter(prediction_times(2:end)-burnout_time, predicted_apogee_altitudes(2:end) - apogee_altitude,2, 'b'); 
ylim([-10 10])


nexttile;
hold on;
scatter(times(1:1000), (x_est(3, 1:1000) - acc(1:1000,1)'))
xline(launch_time, 'g:', 'DisplayName', 'Launch Time');
hold off;
xline(burnout_time, 'g:', 'DisplayName', 'Motor Burnout Time');
drawnow;  % Update plot

nexttile;
hold on;
scatter(times(1:1000), uout(1:1000))
xline(launch_time, 'g:', 'DisplayName', 'Launch Time');
hold off;
xline(burnout_time, 'g:', 'DisplayName', 'Motor Burnout Time');
drawnow;  % Update plot