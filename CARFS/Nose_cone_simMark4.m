function [M3, Cd_nose, T_body, P_body, Rho_body, Re_nose, Rho_nose_avg]=Nose_cone_simMark4(delta_t, hitsuper,L_nose, gamma, Mach, Radius,rho,static_pressure,temp,mu, Swet_nose, Sref_nose,Nose_cone_type)

R = 1716;
%%

% clear all
% Mach =1.32;
% R = 1716;
% 
% 
% diameter = 4.5/12;
% Aft_diameter = 4.5/12;
% L_nose = 18/12;
% L_body = 108/12;
% Root_chord_length = 22/12;
% Tip_chord_length = 3/12;
% fin_height = 4.5/12;
% fin_thickness = 0.25/12;
% length_LE = 17.526/12;
% num_fins = 3;
% LW = 69.8;
% 
% 
% Nose_cone_type = 3;
% Radius = diameter /2;
% gamma =1.4;
% 
% delta_t = 1/400;
% hitsuper = 1;


if Radius > ( 2.8/12)
    Radius1 = Radius*1;
    L_nose1 = L_nose*1;
else
    Radius1 = Radius*2;
    L_nose1 = L_nose*2;
end

i = 1;

c = 1;
theta = zeros((round(L_nose)+1)/.1,1);
y = zeros((round(L_nose1)+1)/.1,1);
x = zeros((round(L_nose1)+1)/.1,1);
y2 = zeros((round(L_nose1)+1)/.1,1);
deflection_angle = zeros((length(y)),1);
while c < L_nose1+1
    if Nose_cone_type ==1 
        Kprime = 0.75;
        %Power series 3/4
        y(i) = Radius1*(2*(x(i)/L_nose1)-Kprime*(x(i)/L_nose1)^2)/(2-Kprime);
        y2(i) = -(Radius1*(2*(x(i)/L_nose1)-Kprime*(x(i)/L_nose1)^2)/(2-Kprime));
    elseif Nose_cone_type == 2      
        %Ogive
        k = 1; %k=1 for tangent ogive
        p = sqrt((L_nose1^2 + Radius1^2)*(((2-k)*L_nose1)^2+(k*Radius1)^2)/(4*(k*Radius1)^2));
        y(i) = sqrt((p^2)-((L_nose1/k)-x(i))^2) - sqrt((p^2)-(L_nose1/k)^2);
        y2(i) = -(sqrt((p^2)-((L_nose1/k)-x(i))^2) - sqrt((p^2)-(L_nose1/k)^2));
    elseif Nose_cone_type == 3       
       % Haak Series Nose Cone
        C = 0;
        y(i) = (Radius1/sqrt(3.141459))*sqrt((acos(1-2*x(i)/L_nose1)) - (sin(2*(acos(1-2*x(i)/L_nose1)))/2)+ C*sin((acos(1-2*x(i)/L_nose1)))*sin((acos(1-2*x(i)/L_nose1)))*sin((acos(1-2*x(i)/L_nose1))));
        y2(i) = -(Radius1/sqrt(3.141459))*sqrt((acos(1-2*x(i)/L_nose1)) - (sin(2*(acos(1-2*x(i)/L_nose1)))/2)+ C*sin((acos(1-2*x(i)/L_nose1)))*sin((acos(1-2*x(i)/L_nose1)))*sin((acos(1-2*x(i)/L_nose1))));
    else
    end
    i = i+1;
    x(i) = x(i-1)+delta_t;
    c = c+delta_t;
end
if Nose_cone_type ==1 
    Kprime = 0.75;
    %Power series 3/4
    y(i) = Radius1*(2*(x(i)/L_nose1)-Kprime*(x(i)/L_nose1)^2)/(2-Kprime);
    y2(i) = -(Radius1*(2*(x(i)/L_nose1)-Kprime*(x(i)/L_nose1)^2)/(2-Kprime));
elseif Nose_cone_type == 2      
   %Ogive
    k = 1; %k=1 for tangent ogive
    p = sqrt((L_nose1^2 + Radius1^2)*(((2-k)*L_nose1)^2+(k*Radius1)^2)/(4*(k*Radius1)^2));
    y(i) = sqrt((p^2)-((L_nose1/k)-x(i))^2) - sqrt((p^2)-(L_nose1/k)^2);
    y2(i) = -(sqrt((p^2)-((L_nose1/k)-x(i))^2) - sqrt((p^2)-(L_nose1/k)^2));
elseif Nose_cone_type == 3       
   % Haak Series Nose Cone
    C = 0;
    y(i) = (Radius1/sqrt(3.141459))*sqrt((acos(1-2*x(i)/L_nose1)) - (sin(2*(acos(1-2*x(i)/L_nose1)))/2)+ C*sin((acos(1-2*x(i)/L_nose1)))*sin((acos(1-2*x(i)/L_nose1)))*sin((acos(1-2*x(i)/L_nose1))));
    y2(i) = -(Radius1/sqrt(3.141459))*sqrt((acos(1-2*x(i)/L_nose1)) - (sin(2*(acos(1-2*x(i)/L_nose1)))/2)+ C*sin((acos(1-2*x(i)/L_nose1)))*sin((acos(1-2*x(i)/L_nose1)))*sin((acos(1-2*x(i)/L_nose1))));
else
end
y = real(y);
y2 = real(y2);

if hitsuper == 1
    figure (1)
    plot(x,y,'b',x,y2,'b'), grid on
    axis([0 L_nose1+20 -20 20]);
    hold on
else
    hold off
end

i_1 = 1;
i_2 = 2;
while i_2 <= length(y)
    
    slope = (y(i_2)-y(i_1))/(x(i_2)-x(i_1));
    defl_theta = atan(slope);
    i_1 = i_1 + 1;
    i_2 = i_2 + 1;
    deflection_angle(i_1) = defl_theta*180/pi;
  
end
deflection_angle(1) = deflection_angle(2);
k = 2;
i = 1;
z = 0;
y_t = 0;
surf_length_nose = 0.0;
surf_length_nose_indv = zeros((round(L_nose1)+1)/.1,1);
d_y = zeros((round(L_nose1)+1)/.1,1);

while z < L_nose1
    
    d_y(i) = y(k)-y(i);
    
    surf_length_nose= surf_length_nose + sqrt(delta_t^2 + d_y(i)^2);
    surf_length_nose_indv(i) = sqrt(delta_t^2 + d_y(i)^2);
    y_t = y_t + d_y(i);
    z = z + delta_t;
    k = k+1;
    i = i+1;
    
end % length across nose cone
t = 0;
arb = 1;
counter = 1;
variable = 0;

%Average first 2 inches of nose cone
while t < 2
    if arb > length(deflection_angle)
        break
    else
    end
    variable = variable + deflection_angle(arb);
    counter = counter +1;
    t = t + delta_t;
    arb = arb + 1;
end
Average_angle_2inch = variable / counter;

nose_cone_angle_deg = Average_angle_2inch;

%% Nose cone CFD
M = zeros(length(y),1);
Rho_nose = zeros(length(y),1);
P_nose = zeros(length(y),1);
T_nose = zeros(length(y),1);

Rho_nose(1) = rho;
P_nose(1) = static_pressure;
T_nose(1) = temp;

M(1) = Mach;

A = nose_cone_angle_deg; %angle of deflection
B = 0.0;
s = 1; %arbitraty shock angle
s = s*pi/180;
ncc=1;

%Oblique shock flow
while B <= A
    % Following explicit relation for deflection angle from
    % Liepmann & Roshko
    % s - shock wave angle
    b = (2/(tan(s))*((Mach^2*sin(s)*sin(s)-1)/((Mach^2*(gamma+cos(2*s))+2))));
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
% 
% if hitsuper == 1
%     %PLOTTING SHOCKWAVE     
%     xx = 0:0.01:20;
%     Shk_wv = tan(s)*xx;
%     Shk_wv2 = -tan(s)*xx;
%     plot(xx,Shk_wv)
%     color('r')
%     hold on
%     plot(xx,Shk_wv2)
%     hold off
% else
%     hold off
% end


Rho_nose2 = rho*(((gamma+1)*Mn1^2)/((gamma-1)*Mn1^2+2));
P_nose2 = static_pressure*(1+((2*gamma/(gamma+1))*(Mn1^2-1)));
T_nose2 = temp*((1 +((2*gamma)/(gamma+1))*(Mn1^2-1))*(((2+(gamma-1)*Mn1^2))/((gamma+1)*Mn1^2)));

a_nose = sqrt(gamma*R*T_nose2);
Re_nose = (Rho_nose2 * M2*a_nose * 2)/(mu*((T_nose2/temp)^1.5)*((temp+198.72)/(T_nose2+198.72)));
Cf_nose = 0.032*(5/2)^0.2;
Cd_nose = 0.9*Cf_nose*Swet_nose/Sref_nose;

Reduced_deflection = zeros(length(y)+1,1);


p = 1;
i = 2;

Reduced_deflection(p) = abs(Average_angle_2inch - deflection_angle(counter+1));

M3 = 1.0;
M(i) = M2;

Rho_nose(i) = Rho_nose2;
P_nose(i) = P_nose2;
T_nose(i) = T_nose2;

%% Prandtl-Meyer Flow
% Calculating prandtl-meyer after the first summarized angle
v0 = (sqrt((gamma+1)/(gamma-1)) * atan(sqrt(((gamma-1)/(gamma+1))*(M(i)^2-1)))-atan(sqrt(M(i)^2-1)));
    v0 = v0*180/pi;
    v3 = v0 + Reduced_deflection(p);
    %fprintf('reduced deflection = %.2f\n v3 = %.2f\n',Reduced_deflection(p),v3);
    v_1 = 0;
    
while v_1 <= v3
        %%*** Nasa files on Prandtl-Meyer Expansive Waves **%%
        % also on p.170 gas dynamics book
        v_1 =  sqrt((gamma+1)/(gamma-1)) *atan(sqrt(((gamma-1)/(gamma+1))*(M3^2-1)))-atan(sqrt(M3^2-1));
        v_1 = v_1*180/pi;
        M3 = M3+0.001;

end 
distance_along_nosecone = arb*delta_t;

i = i+1;
M(i) = M3;

T_nose3 = T_nose2*(((1+((gamma-1)/2)*M2^2))/(1+((gamma-1)/2)*(M3^2)));
P_nose3 = P_nose2*(((1+((gamma-1)/2)*M2^2))/(1+((gamma-1)/2)*(M3^2)))^(gamma/(gamma-1));
Rho_nose3 = Rho_nose2*(((1+((gamma-1)/2)*M2^2))/(1+((gamma-1)/2)*(M3^2)))^(gamma/(gamma-1));

Rho_nose(i) = Rho_nose3;
P_nose(i) = P_nose3;
T_nose(i) = T_nose3;

p = p+1;
counter = counter +1;
Reduced_deflection(p) = abs(deflection_angle(counter+1) - deflection_angle(counter));
Bigcounter = 1;
v0 = (sqrt((gamma+1)/(gamma-1)) * atan(sqrt(((gamma-1)/(gamma+1))*(M(i)^2-1)))-atan(sqrt(M(i)^2-1)));    
v0 = v0*180/pi;   
defl_length = length(deflection_angle);
% Handles every section past the summaraized angle

%% 
while Reduced_deflection(p) > 0 
    
%     fprintf('v0 = %0.6f\n',v0);
%     fprintf('reduced deflection = %.6f\n',Reduced_deflection(p));
   
    v3 = v0 + Reduced_deflection(p);
   % fprintf('v3 = %0.6f\n',v3);
    
    %fprintf('V3 = %0.2f\n',v3);
    distance_along_nosecone = distance_along_nosecone + delta_t;
    v_1 = 0;
    M3 = 1.0;
    
   
    while v_1 < v3
        %%*** Nasa files on Prandtl-Meyer Expansive Waves **%%
        % also on p.170 gas dynamics book
        v_1 =  sqrt((gamma+1)/(gamma-1)) *atan(sqrt(((gamma-1)/(gamma+1))*(M3^2-1)))-atan(sqrt(M3^2-1));
        v_1 = v_1*180/pi;
        M3 = M3 + 0.001;
        Bigcounter = Bigcounter +1;
    end
    v0 = v3;
    %fprintf('M3 = %.6f\n\n',M3);
    i = i+1;
    M(i) = M3; %4th mach number in the matrix
    
    T_nose(i) = T_nose(i-1)*(((1+((gamma-1)/2)*M(i-1)^2))/(1+((gamma-1)/2)*(M3^2)));
    P_nose(i) = P_nose(i-1)*(((1+((gamma-1)/2)*M(i-1)^2))/(1+((gamma-1)/2)*(M3^2)))^(gamma/(gamma-1));
    Rho_nose(i) = Rho_nose(i-1)*(((1+((gamma-1)/2)*M(i-1)^2))/(1+((gamma-1)/2)*(M3^2)))^(gamma/(gamma-1));
    
    
    
    p = p+1;
    counter = counter +1;
    deflection_angle(defl_length+1) = 0;
    deflection_angle(defl_length+2) = 0;
    Reduced_deflection(p) = abs(deflection_angle(counter+1) - deflection_angle(counter));
  
    a_nose = sqrt(gamma*R*T_nose(i));
    Re_nose = (Rho_nose(i) * M3*a_nose * delta_t)/(mu*((T_nose(i)/temp)^1.5)*((temp+198.72)/(T_nose(i)+198.72))); 
    
    Cf_nose = 0.032*(5/delta_t)^0.2;
    
    
    Cd_nose = Cd_nose + 0.9*Cf_nose*Swet_nose/Sref_nose;
   
end
u = 1;
Rho_Nose = 0;
while u <= i
    Rho_Nose = Rho_Nose + Rho_nose(u);
    u = u+1;
end

Rho_nose_avg = Rho_Nose/i;
Re_nose = Rho_nose_avg*mean(M)*sqrt(gamma*R*mean(T_nose))*L_nose1/(mu*(((mean(T_nose)/temp)^1.5)*((temp+198.72)/(mean(T_nose)+198.72))));

Cf_nose = 0.074/(Re_nose^0.2);

Cd_nose = Cf_nose*Swet_nose/Sref_nose;


T_body = T_nose(i);
P_body = P_nose(i);
Rho_body = Rho_nose(i);


end
% fprintf('Distance along nosecone = %.2f\n',distance_along_nosecone);
% fprintf('M3 = %0.6f\n',M3);  
