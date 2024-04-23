%% Script to plot the predicted apogee

sim.Apogee = max(sim.LLA(:,3));

plots.apogee = figure;
hold on;
yline(sim.Apogee,'DisplayName','True Apogee');
plot(sim.time,apest(1,:),'DisplayName','Predicted Apogee');
plot(sim.time,testAp(1,:),'DisplayName','Predicted Jack Apogee');
plot(sim.time,sim.LLA(:,3),'DisplayName','True Altitude');
plot(sim.time,llaEst(3,:),'DisplayName','Estimated Altitude');
xlabel("Time [s]");
legend;
hold off;

