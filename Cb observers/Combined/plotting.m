% Load results
load('observers.mat');

% Plot states
figure;
subplot(3, 1, 1);
plot(time, ekf_states(:, 1), 'b', time, ukf_states(:, 1), 'r');
xlabel('Time (s)');
ylabel('Altitude (m)');
legend('EKF', 'UKF');
title('Altitude over time');

subplot(3, 1, 2);
plot(time, ekf_states(:, 2), 'b', time, ukf_states(:, 2), 'r');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend('EKF', 'UKF');
title('Velocity over time');

subplot(3, 1, 3);
plot(time, ekf_states(:, 3), 'b', time, ukf_states(:, 3), 'r');
xlabel('Time (s)');
ylabel('Ballistic Coefficient (m^2/kg)');
legend('EKF', 'UKF');
title('Ballistic Coefficient over time');

% Plot covariance (Optional)
figure;
subplot(3, 1, 1);
plot(time, squeeze(ekf_covariances(:, 1, 1)), 'b', time, squeeze(ukf_covariances(:, 1, 1)), 'r');
xlabel('Time (s)');
ylabel('Covariance of Altitude');
legend('EKF', 'UKF');
title('Covariance of Altitude over time');

subplot(3, 1, 2);
plot(time, squeeze(ekf_covariances(:, 2, 2)), 'b', time, squeeze(ukf_covariances(:, 2, 2)), 'r');
xlabel('Time (s)');
ylabel('Covariance of Velocity');
legend('EKF', 'UKF');
title('Covariance of Velocity over time');

subplot(3, 1, 3);
plot(time, squeeze(ekf_covariances(:, 3, 3)), 'b', time, squeeze(ukf_covariances(:, 3, 3)), 'r');
xlabel('Time (s)');
ylabel('Covariance of Ballistic Coefficient');
legend('EKF', 'UKF');
title('Covariance of Ballistic Coefficient over time');
