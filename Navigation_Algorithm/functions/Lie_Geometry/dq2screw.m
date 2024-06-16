function [ang,p,ax,m] = dq2screw(dq)
%  DQ2SCREW Calculates screw parameters from unit dual quaternion.
%   Together ANG and P form the dual angle, AX and M form the dual axis
% Real part r and dual part d
r=dq(1:4);
d=dq(5:8);
% Translation
t=2*quatmultiply(d,quatconj(r));
t=t(2:4);
% Angle-axis
axang=quat2axang(r);
ang=axang(4);
ax=axang(1:3);
% In case of pure translation (screw axis at infinity)
if abs(ang)<eps && norm(t)~=0
    ax=t/norm(t);
end
% Pitch
p=dot(t,ax);
% Moment
m=1/2*(cross(t,ax)+(t-p*ax)*cot(ang/2));
end