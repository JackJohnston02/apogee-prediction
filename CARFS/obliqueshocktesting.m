
% gamma = 1.4;
% Mach = 1.27;
% A = 5.5067; %angle of deflection
% B = 0.0;
% s = 1; %arbitraty shock angle
% s = s*pi/180;
% ncc=1;
% while B <= A
%     % Following explicit relation for deflection angle from
%     % Liepmann & Roshko
%     b = (2/(tan(s))*((Mach^2*sin(s)*sin(s)-1)/((Mach^2*(gamma+cos(2*s))+2))));
%     % Nasa equation for oblique shock relations
%     %     b =  tan(s)*(((gamma+1)*Mach^2)/(2*(Mach^2*sin(s)*sin(s)-1))-1);
%     %     B = acot(b)*180/pi;
%     B = atan(b)*180/pi;
%     s = s+pi/180;
%     ncc=ncc+1;
%     fprintf('B: %.2f\n',B);
%     fprintf('s: %.2f\n',s*180/pi);
% end
% s = s*180/pi -2;
% Mn1 = Mach*sin(s*pi/180);
% Mn2 = sqrt((Mn1^2+ (2/(gamma-1)))/(((2*gamma)/(gamma-1))*Mn1^2-1));%(4.10) MOdern compressible flow
% s = s*pi/180;
% A = A*pi/180;
% M2 = Mn2/sin(s-A);
% s = s*180/pi;
% A = A*180/pi;
% fprintf('Wedge angle: %.2f deg. Shock angle: %.2f deg.\n',A,s); 
% fprintf('B: %.2f',B);

row = 1;

Mach = 4;
gamma = 1.4;
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