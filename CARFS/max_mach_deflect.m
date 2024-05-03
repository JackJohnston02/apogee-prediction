 function Theta_max = max_mach_deflect(Mach,gamma)
row = 1;

% oblique_shock_min = oblique_shock_min*180/pi +2;
Oblique_shock_table = zeros(2000,2);
oblique_shock_min = asin(1/Mach);
oblique_shock_min = oblique_shock_min*180/pi;

    while oblique_shock_min < 90
        oblique_shock_min = oblique_shock_min*pi/180;
        Theta = atan((2/(tan(oblique_shock_min))*((Mach^2*sin(oblique_shock_min)*sin(oblique_shock_min)-1)/((Mach^2*(gamma+cos(2*oblique_shock_min))+2)))));
        Theta = Theta *180/pi;
        oblique_shock_min = oblique_shock_min*180/pi;
        Oblique_shock_table(row , :) = [oblique_shock_min Theta];
        oblique_shock_min = oblique_shock_min*pi/180;
        row = row + 1;

        oblique_shock_min = oblique_shock_min + pi/180;
        oblique_shock_min = oblique_shock_min*180/pi;
    end

Oblique_shock_table = Oblique_shock_table(1:(row-1),:);
Theta_max = max(Oblique_shock_table(:,2));

end

