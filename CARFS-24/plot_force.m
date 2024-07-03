    plot(t, Rocket.x(:,3)*7.7 + 9.81*7.7, 'LineWidth', 2);
    xlim([3.3,20]);
    xline(5.01);
    title('Drag Force vs Time');
    xlabel('Time (s)');
    ylabel('Force (N)');
    grid on;  % add a grid