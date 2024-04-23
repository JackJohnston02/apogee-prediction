%% Script to plot RocketPy Data
llaEst = zeros(3,sim.numTimeSteps);
for i= 1:sim.numTimeSteps
    [llaEst(1,i),llaEst(2,i),llaEst(3,i)] = ned2geodetic(x_estNED(i,1),x_estNED(i,2),x_estNED(i,3),sim.LLA(1,1),sim.LLA(1,2),sim.LLA(1,3),nav.wgs84);
end

% LLA PLot using Google Earth
plots.LLA = uifigure;
uif = geoglobe(plots.LLA,'NextPlot','add');
geoplot3(uif,sim.LLA(:,1),sim.LLA(:,2),sim.LLA(:,3),'Color','b');
hold(uif,'on');
geoplot3(uif,sensor.gnss.LLA_meas_samp(:,1),sensor.gnss.LLA_meas_samp(:,2),sensor.gnss.LLA_meas_samp(:,3),'ro');
geoplot3(uif,llaEst(1,:),llaEst(2,:),llaEst(3,:),'Color','g');
hold(uif,'off');

% ENU Position
plots.xNED = figure;
hold on;
grid minor;
xlabel('X ENU [m]');
ylabel('Y ENU [m]');
zlabel('Z ENU [m]');
plot3(sim.x_ned(:,1),sim.x_ned(:,2),sim.x_ned(:,3),'Color','b','DisplayName','ENU Position');
plot3(x_estNED(:,1),x_estNED(:,2),x_estNED(:,3),'Color','r','DisplayName','Nav Estimate');
hold off;
legend;
axis equal;

% NED Position
plots.ned_position = figure;
subplot(3,1,1);
hold on;
plot(sim.time,sim.x_ned(:,1),'Color','b','DisplayName','True X NED');
plot(sim.time,x_estNED(:,1),'Color','g','DisplayName','Est X NED');
grid minor;
legend;
xlabel('Time [s]');
ylabel('NED X Position');
hold off;
subplot(3,1,2);
hold on;
plot(sim.time,sim.x_ned(:,2),'Color','b','DisplayName','True Y NED');
plot(sim.time,x_estNED(:,2),'Color','g','DisplayName','Est Y NED');
grid minor;
legend;
xlabel('Time [s]');
ylabel('NED Y Position');
hold off;
subplot(3,1,3);
hold on;
plot(sim.time,sim.x_ned(:,3),'Color','b','DisplayName','True Z NED');
plot(sim.time,x_estNED(:,3),'Color','g','DisplayName','Est Z NED');
grid minor;
legend;
xlabel('Time [s]');
ylabel('NED Z Position');
hold off;

% Angular Rates
plots.omega_b = figure;
subplot(3,1,1);
hold on;
plot(sim.time,sim.omega_b(:,1),'Color','b','DisplayName','omega x (Roll)');
plot(sim.time,sensor.gyro.omega_b_meas_samp(:,1),'Color','r','DisplayName','Roll Rate Meas');
plot(sim.time,west(1,:),'Color','g','DisplayName','omega x Est');
grid minor;
legend;
xlabel('Time [s]');
ylabel('Angular Rate [rad/s]');
hold off;
subplot(3,1,2);
hold on;
plot(sim.time,sim.omega_b(:,2),'Color','b','DisplayName','omega y (Pitch)');
plot(sim.time,sensor.gyro.omega_b_meas_samp(:,2),'Color','r','DisplayName','Pitch Rate Meas');
plot(sim.time,west(2,:),'Color','g','DisplayName','omega y Est');
grid minor;
legend;
xlabel('Time [s]');
ylabel('Angular Rate [rad/s]');
hold off;
subplot(3,1,3);
hold on;
plot(sim.time,sim.omega_b(:,3),'Color','b','DisplayName','omega z (Yaw)');
plot(sim.time,sensor.gyro.omega_b_meas_samp(:,3),'Color','r','DisplayName','Yaw Rate Meas');
plot(sim.time,west(3,:),'Color','g','DisplayName','omega z Est');
grid minor;
legend;
xlabel('Time [s]');
ylabel('Angular Rate [rad/s]');
hold off;

% Angular Accelerations
plots.alpha_b = figure;
subplot(3,1,1);
hold on;
plot(sim.time,sim.alpha_b(:,1),'Color','b','DisplayName','alpha x (Roll)');
grid minor;
legend;
xlabel('Time [s]');
ylabel('Angular Acceleration [rad/s]');
hold off;
subplot(3,1,2);
hold on;
plot(sim.time,sim.alpha_b(:,2),'Color','b','DisplayName','alpha y (Pitch)');
grid minor;
legend;
xlabel('Time [s]');
ylabel('Angular Acceleration [rad/s]');
hold off;
subplot(3,1,3);
hold on;
plot(sim.time,sim.alpha_b(:,3),'Color','b','DisplayName','alpha z (Yaw)');
grid minor;
legend;
xlabel('Time [s]');
ylabel('Angular Acceleration [rad/s]');
hold off;

% Linear Velocities
plots.v_enu = figure;
subplot(3,1,1);
hold on;
plot(sim.time,sim.v_enu(:,1),'Color','b','DisplayName','x');
grid minor;
legend;
xlabel('Time [s]');
ylabel('Linear Velocity [m/s]');
hold off;
subplot(3,1,2);
hold on;
plot(sim.time,sim.v_enu(:,2),'Color','b','DisplayName','y');
grid minor;
legend;
xlabel('Time [s]');
ylabel('Linear Velocity [m/s]');
hold off;
subplot(3,1,3);
hold on;
plot(sim.time,sim.v_enu(:,3),'Color','b','DisplayName','z');
grid minor;
legend;
xlabel('Time [s]');
ylabel('Linear Velocity [m/s]');
hold off;

% Linear Accelerations
plots.a_enu = figure;
subplot(3,1,1);
hold on;
plot(sim.time,sim.a_enu(:,1),'Color','b','DisplayName','x');
grid minor;
legend;
xlabel('Time [s]');
ylabel('Linear Acceleration [m/s^2]');
hold off;
subplot(3,1,2);
hold on;
plot(sim.time,sim.a_enu(:,2),'Color','b','DisplayName','y');
grid minor;
legend;
xlabel('Time [s]');
ylabel('Linear Acceleration [m/s^2]');
hold off;
subplot(3,1,3);
hold on;
plot(sim.time,sim.a_enu(:,3),'Color','b','DisplayName','z');
grid minor;
legend;
xlabel('Time [s]');
ylabel('Linear Acceleration [m/s^2]');
hold off;

% Attitude using NED reference frame
% Rotate reference vector for 3D plot
e3 = [0 0 1];
for i = 1:size(sim.time,1)
    sim.s3(i,:) = quatrotate(sim.qned(:,i)',e3);
end

plots.attitude = figure;
grid minor;
hold on;
[x,y,z] = sphere(30);
surf(x,y,z,'FaceAlpha',0.1,'DisplayName','Attitude Sphere');
xlabel('X NED [m]');
ylabel('Y NED [m]');
zlabel('Z NED [m]');
plot3(sim.s3(:,1),sim.s3(:,2),sim.s3(:,3),'Color','b','DisplayName','Attitude in NED Frame','LineWidth',2);
plot3(s3est(:,1),s3est(:,2),s3est(:,3),'Color','r','DisplayName','Attitude Estimate in NED Frame','LineWidth',2);
hold off;
legend;





