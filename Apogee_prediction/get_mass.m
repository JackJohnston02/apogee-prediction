function [mass] = get_mass(t,rocket_path)
% Open the file
fileID = fopen(rocket_path, 'r');

% Read the data
dryMass = fscanf(fileID, 'Dry Mass: %f\n', 1);
wetMass = fscanf(fileID, 'Wet Mass: %f\n', 1);
burnTime = fscanf(fileID, 'Burn Time: %f\n', 1);

% Close the file
fclose(fileID);

% Display the data
mass = wetMass - (t/burnTime) * (wetMass - dryMass);

end
