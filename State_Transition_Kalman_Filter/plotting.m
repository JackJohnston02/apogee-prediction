% Plotting for constant velocity model
if STFilter.type == "CV"
    statePlot = figure('Name','Constant Velocity Filter States');
    
    subplot(2,1, 1);
    hold on;
    plot(data_struct.timestamp, data.baro_altitude);
    plot(times, x_est(1,:));
    
    % Plot state transition times
    xline(StateMachine.padToBurning, "--r");
    xline(StateMachine.burningToCoasting, "--r");
    xline(StateMachine.coastingTodescent, "--r");
    xline(StateMachine.descentToLanded, "--r");
    yline(StateMachine.apogeeAltitude, "--r")
    
    % Add text labels to vertical lines
    ylim_top = ylim;
    text(StateMachine.padToBurning, ylim_top(2), 'Launch', 'Color', 'r', ...
        'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'center', 'FontSize', 8);
    text(StateMachine.burningToCoasting, ylim_top(2), 'Burnout', 'Color', 'r', ...
        'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'center', 'FontSize', 8);
    text(StateMachine.coastingTodescent, ylim_top(2), 'Apogee', 'Color', 'r', ...
        'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'center', 'FontSize', 8);
    text(StateMachine.descentToLanded, ylim_top(2), 'Landed', 'Color', 'r', ...
        'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'center', 'FontSize', 8);
    
    grid on;
    grid minor;
    
    legend("Barometer Readings", "Estimates");
    hold off
    
    subplot(2,1, 2);
    hold on;
    plot(times, x_est(2,:));
    
    % Plot state transition times
    xline(StateMachine.padToBurning, "--r");
    xline(StateMachine.burningToCoasting, "--r");
    xline(StateMachine.coastingTodescent, "--r");
    xline(StateMachine.descentToLanded, "--r");
    
    grid on;
    grid minor;
    
    hold off;
end


% Plotting for constant acceleration model
if STFilter.type == "CA"
    statePlot = figure('Name','Constant Acceleration Filter States');

    subplot(3,1, 1);
    hold on;
    plot(data_struct.timestamp, data.baro_altitude);
    plot(times, x_est(1,:));
    legend("Barometer Readings", "Estimates");

    % Plot state transition times
    xline(StateMachine.padToBurning, "--r");
    xline(StateMachine.burningToCoasting, "--r");
    xline(StateMachine.coastingTodescent, "--r");
    xline(StateMachine.descentToLanded, "--r");
    yline(StateMachine.apogeeAltitude, "-- r")
    grid on;
    grid("minor");
    hold off
    
    subplot(3,1, 2);
    hold on;
    plot(times, x_est(2,:));
    hold off;

    subplot(3,1, 3);
    hold on;
    plot(data_struct.timestamp, data_struct.imu_accZ);
    plot(times, x_est(3,:));
    legend("Barometer Readings", "Estimates");
    hold off;
end