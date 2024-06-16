function [AdjX] = se2_3_adjoint(X)

% Select Components
R = X(1:3,1:3);
p = X(1:3,5);
v = X(1:3,4);

O3 = zeros(3,3);

AdjX = [R O3 O3;...
         so3_wedge(v)*R R O3;...
         so3_wedge(p)*R O3 R];

end

