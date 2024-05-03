function [state] = state_update(Rocket)
    state = Rocket.state;

    % Detect launch and switch  
    if Rocket.state == "pad" && Rocket.x(3) > 10
        state = "launched";
    end

    % Detect burnout and switch  
    if Rocket.state == "launched" && Rocket.x(3)< 0 
        state = "burntout";
    end

    % Detect apogee and switch  
    if Rocket.state == "burntout" && Rocket.x(2)< 0 
        state = "descent";
    end

    % Detect apogee and switch  
    if Rocket.state == "descent" && Rocket.x(1)< 0
        state = "landed";
    end

end

