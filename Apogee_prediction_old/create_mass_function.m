function mass_function = create_mass_function(rocket_path)
    % Read mass data from file
    [dryMass, wetMass, burnTime] = read_mass_data(rocket_path);
    
    % Create function handle for mass calculation
    mass_function = @(t) calculate_mass(t, dryMass, wetMass, burnTime);
end

function [dryMass, wetMass, burnTime] = read_mass_data(rocket_path)
    % Open the file
    fileID = fopen(rocket_path, 'r');

    % Check if file exists
    if fileID == -1
        error('File not found');
    end

    % Read the data
    dryMass = fscanf(fileID, 'Dry Mass: %f\n', 1);
    wetMass = fscanf(fileID, 'Wet Mass: %f\n', 1);
    burnTime = fscanf(fileID, 'Burn Time: %f\n', 1);

    % Close the file
    fclose(fileID);
end

function mass = calculate_mass(t, dryMass, wetMass, burnTime)
    % Calculate mass at time t
    mass = wetMass - (t / burnTime) * (wetMass - dryMass);
end
