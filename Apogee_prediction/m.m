function [mass] = m(t)

dry_mass = 3;
wet_mass = 5;
burn_time = 3;


masses = dry_mass * ones(100,1);

for i = 1:burn_time
        masses(i) = ((wet_mass - dry_mass)/burn_time)*i + masses(i);
end


mass = masses(t);
end

