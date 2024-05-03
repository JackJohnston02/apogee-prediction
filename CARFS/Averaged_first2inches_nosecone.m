function Average_angle_2inch = Averaged_first2inches_nosecone(L_nose,Radius,delta_t,Nose_cone_type,k)


% L_nose = 21;
% Radius = 5.5/2;
% delta_t = 1/200;
% Nose_cone_type = 2;

if Radius > (2.8/12)
    Radius1 = Radius*1;
    L_nose1 = L_nose*1;
else
    Radius1 = Radius*2;
    L_nose1 = L_nose*2;
end

i = 1;
i_1 = 1;
i_2 = 2;
c = 1;
Kprime = 0.75;
theta = zeros((round(L_nose1)+1)/.1,1);
y = zeros((round(L_nose1)+1)/.01,1);
x = zeros((round(L_nose1)+1)/.01,1);
y2 = zeros((round(L_nose1)+1)/.01,1);
deflection_angle = zeros((length(y)),1);
while c < L_nose1+1
   if Nose_cone_type ==1 
        Kprime = 0.75;
        %Power series 3/4
        y(i) = Radius1*(2*(x(i)/L_nose1)-Kprime*(x(i)/L_nose1)^2)/(2-Kprime);
        y2(i) =-(Radius1*(2*(x(i)/L_nose1)-Kprime*(x(i)/L_nose1)^2)/(2-Kprime));
    elseif Nose_cone_type == 2      
        %Ogive
        k = 1; %k=1 for tangent ogive
        p = sqrt((L_nose1^2 + Radius1^2)*(((2-k)*L_nose1)^2+(k*Radius1)^2)/(4*(k*Radius1)^2));
        y(i) = sqrt((p^2)-((L_nose1/k)-x(i))^2) - sqrt((p^2)-(L_nose1/k)^2);
        y2(i) = -(sqrt((p^2)-((L_nose1/k)-x(i))^2) - sqrt((p^2)-(L_nose1/k)^2));
   elseif Nose_cone_type == 3  
        ogive =0;
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
    y2(i) = -((Radius1/sqrt(3.141459))*sqrt((acos(1-2*x(i)/L_nose1)) - (sin(2*(acos(1-2*x(i)/L_nose1)))/2)+ C*sin((acos(1-2*x(i)/L_nose1)))*sin((acos(1-2*x(i)/L_nose1)))*sin((acos(1-2*x(i)/L_nose1)))));
 
else
end


y = real(y);
%y2 = real(y2);
theta = real(theta);
plot(x,y,'b',x,y2,'b'), grid on
axis([0 L_nose+20 -20 20]);

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
while t < 2.000
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

end
