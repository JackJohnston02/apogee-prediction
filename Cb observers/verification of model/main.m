x = [0;340;1500];

g = -9.81;
rho = 1.293;

A = [x(1); g - (rho * x(2)^2)/2*x(3);0];

xlog = [];
tlog = [];
t = 0;
dt = 0.001;

while t < 10 && x(2) > 0

    A = [x(2); g - (rho * x(2)^2)/(2*x(3));0];
    x = x + A*dt;
    
    t = t + dt;
    xlog = [xlog, x];
    tlog = [tlog, t];
end

figure;
hold on
subplot(3,1,1);
plot(tlog, xlog(1,:))

subplot(3,1,2);
plot(tlog, xlog(2,:))

subplot(3,1,3);
plot(tlog, xlog(3,:))