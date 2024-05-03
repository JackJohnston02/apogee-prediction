%Oblique shock flow
clc 
clear all

Mach = 4.0;
A = 11.41;%5.71
theta = 5.71;
del_theta = 2*theta;
B = 0.0;
gamma =1.4;
R = 287;

s = 1; %arbitraty shock angle
s = s*pi/180;
ncc=1;
static_pressure = 1; %atm
temp = 270; %Kelvin
rho = (static_pressure)*temp/(R);

while B <= A
    % Following explicit relation for deflection angle from
    % Liepmann & Roshko
    % s - shock wave angle
    b = (2/(tan(s)))*((Mach^2*sin(s)*sin(s)-1)/(((Mach^2)*(gamma+cos(2*s))+2)));
    % Nasa equation for oblique shock relations
%     b =  tan(s)*(((gamma+1)*Mach^2)/(2*(Mach^2*sin(s)*sin(s)-1))-1);
%     B = acot(b)*180/pi;
    B = atan(b)*180/pi;
    s = s+((pi/180)/100);
    ncc=ncc+1;
%    fprintf('Mach: %.2f shock angle:%.2f deg.\n',Mach,s*180/pi);
%    fprintf('B: %.2f deg.\n',B);

end
s = s*180/pi; %the minus 2 is a correction factor to the table results
Mn1 = Mach*sin(s*pi/180);
Mn2 = sqrt((Mn1^2+ (2/(gamma-1)))/(((2*gamma)/(gamma-1))*Mn1^2-1));%(4.10) MOdern compressible flow
s = s*pi/180;
A = A*pi/180;
M2 = Mn2/sin(s-A);

s = s*180/pi;
A = A*180/pi;


Rho_2 = rho*(((gamma+1)*Mn1^2)/((gamma-1)*Mn1^2+2));
P_2 = static_pressure*(1+((2*gamma/(gamma+1))*(Mn1^2-1)));
T_2 = temp*((1 +((2*gamma)/(gamma+1))*(Mn1^2-1))*(((2+(gamma-1)*Mn1^2))/((gamma+1)*Mn1^2)));

M3 = 1.0;
% Calculating prandtl-meyer after the first summarized angle
v2 = (sqrt((gamma+1)/(gamma-1)) * atan(sqrt(((gamma-1)/(gamma+1))*(M2^2-1)))-atan(sqrt(M2^2-1)));
    v2 = v2*180/pi;
    v3 = v2 + del_theta;
    %fprintf('reduced deflection = %.2f\n v3 = %.2f\n',Reduced_deflection(p),v3);
    v_1 = 0;
    
while v_1 <= v3
        %%*** Nasa files on Prandtl-Meyer Expansive Waves **%%
        % also on p.170 gas dynamics book
        v_1 =  sqrt((gamma+1)/(gamma-1)) *atan(sqrt(((gamma-1)/(gamma+1))*(M3^2-1)))-atan(sqrt(M3^2-1));
        v_1 = v_1*180/pi;
        M3 = M3+0.001;

end 
T_3 = T_2*(((1+((gamma-1)/2)*M2^2))/(1+((gamma-1)/2)*(M3^2)));
P_3 = P_2*(((1+((gamma-1)/2)*M2^2))/(1+((gamma-1)/2)*(M3^2)))^(gamma/(gamma-1));
Rho_3 = Rho_2*(((1+((gamma-1)/2)*M2^2))/(1+((gamma-1)/2)*(M3^2)))^(gamma/(gamma-1));


L_prime = (P_4)*(0.5/(cos(theta)))*cos(alpha+theta) + (P_5











