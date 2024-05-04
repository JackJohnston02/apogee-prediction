function [state] = state_update(Rocket)
%% Function to update the state machine of the rocket throughout flight stages
% Programmer(s): Jack Johnston
% copyright © 2024
%% TODO
    % log time of transition for each stage

%% Code
    state = Rocket.state;

    % Detect launch and switch  
    if Rocket.state == "pad" && Rocket.x(end,3) > 10
        state = "burning";
    end

    % Detect burnout and switch  
    if Rocket.state == "burning" && Rocket.x(end,3)< 0 
        state = "burntout";
    end

    % Detect apogee and switch  
    if Rocket.state == "burntout" && Rocket.x(end,2)< 0 
        state = "descent";
    end

    % Detect apogee and switch  
    if Rocket.state == "descent" && Rocket.x(end,1)< 0
        state = "landed";
    end

end

