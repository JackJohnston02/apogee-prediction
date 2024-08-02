% Plotting states
figure;
subplot(3,1,1);
hold on;
plot(times, x_est(1,:), 'b', 'LineWidth', 0.5);
scatter(apogee_log(1,:), apogee_log(2,:))
scatter(data_struct.timestamp, data_struct.baro_altitude, 4, "LineWidth", 0.1);
xlabel('Time (s)');
ylabel('Altitude (m)');
title('Estimated Altitude vs Actual Altitude');
legend('Estimated Altitude', 'Actual Altitude');
ylim([200, 900]);
hold off;

subplot(3,1,2);
plot(times, x_est(2,:), 'b', 'LineWidth', 2);
hold on;
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Estimated Velocity');
legend('Estimated Velocity');

subplot(3,1,3);
plot(times, x_est(3,:), 'b', 'LineWidth', 0.5);
hold on;
scatter(data_struct.timestamp, data_struct.imu_accZ, 4, "LineWidth", 0.1);
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
title('Estimated Acceleration vs Actual Acceleration');
legend('Estimated Acceleration', 'Actual Acceleration');



figure;
scatter(apogee_log(1,:), apogee_log(2,:) - 789)
yline(0);
ylim([-100, 100]);

