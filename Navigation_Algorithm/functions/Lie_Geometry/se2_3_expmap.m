function [SE2_3] = se2_3_expmap(se2_3)

% Select Elements
phi = se2_3(1:3);
v = se2_3(4:6);
p = se2_3(7:9);

% Norm of Rotation
nphi = norm(phi);

expPhi = so3_expmap(phi);
Jphi = so3_leftJacobian(phi);

% Perform Exponential Map
SE2_3 = [expPhi Jphi*v Jphi*p;...
         zeros(1,3) 1 0;...
         zeros(1,3) 0 1];

end

