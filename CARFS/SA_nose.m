function Swet_nose = SA_nose(Radius,L_nose,Nose_cone_type)

% Kprime = 0.75;
pi = 3.14159;
syms x
if Nose_cone_type ==1 
    Kprime = 0.75;
    %Power series 3/4
    y = Radius*(2*(x/L_nose)-Kprime*(x/L_nose)^2)/(2-Kprime);   
elseif Nose_cone_type == 2      
    k = 1; %k=1 for tangent ogive,
    p = sqrt((L_nose^2 + Radius^2)*(((2-k)*L_nose)^2+(k*Radius)^2)/(4*(k*Radius)^2));
    y = sqrt((p^2)-((L_nose/k)-x)^2) - sqrt((p^2)-(L_nose/k)^2);
elseif Nose_cone_type == 3       
    %Haak Series Nose Cone
    C = 0;
    y = (Radius/sqrt(3.141459))*sqrt((acos(1-2*x/L_nose)) - (sin(2*(acos(1-2*x/L_nose)))/2)+ C*sin((acos(1-2*x/L_nose)))*sin((acos(1-2*x/L_nose)))*sin((acos(1-2*x/L_nose))));
else
end


 dydx = diff(y);
 
 fun = 2*pi*y*(sqrt(1+dydx^2));
 intfun = int(fun,0,L_nose);
 Swet_nose = double(intfun);

%end
