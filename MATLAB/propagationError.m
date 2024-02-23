function [] = propagationError(actual,predicted, times)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    
    %CLOSE PLOT FROM OTHER CODE
    close all;
    errors = [];
    length(times(:))
    
    for k = 1:length(times(:))
        errors(end+1) = 100*((predicted(k) - actual)/actual);
    end
    errors(1) = 0;
    length(errors(:))
    hold on;    
    scatter(times(:), errors(:), 4, 'r', 'x', 'DisplayName', 'Predicted Apogees');
    
    % Add major grid lines
    grid on;
    
    % Add minor grid lines
    grid minor;
    
    hold off;
    drawnow;
end
