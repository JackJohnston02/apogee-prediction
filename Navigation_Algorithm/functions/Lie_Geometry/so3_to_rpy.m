function [rpy] = so3_to_rpy(R)
%SO3_TO_RPY Convert a rotation matrix to RPY Euler angles
%
% Syntax:  [rpy] = so3_to_rpy(Rot)
%
% Inputs:
%    Rot - rotation matrix
%
% Outputs:
%    rpy - roll, pitch, yaw

TOL = 1e-9;

pitch = atan2(-R(3, 1), sqrt(R(1, 1)^2 + R(2, 1)^2));

if norm(pitch - pi/2) < TOL
    yaw = 0;
    roll = atan2(R(1, 2), R(2, 2));
elseif norm(pitch + pi/2.) < 1e-9
    yaw = 0.;
    roll = -atan2(R(1, 2), R(2, 2));
else
    sec_pitch = 1. / cos(pitch);
    yaw = atan2(R(2, 1) * sec_pitch, R(1, 1)* sec_pitch);
    roll = atan2(R(3, 2) * sec_pitch, R(3, 3) * sec_pitch);
end
rpy = [roll; pitch; yaw];
end