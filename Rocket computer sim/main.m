clear all

dt = 0.05; %integration step size 
t = 0:dt:100;
cd = 1;
flight.acceleration = zeros(1,length(t));
flight.velocity = zeros(1,length(t));
flight.displacement = zeros(1,length(t));
rocket.mass = 5;
rocket.area = 1;
rocket.cd = 1; 

for i=1:(length(t)-1)    
    flight.acceleration(i+1) = acceleration_eq(flight.velocity(i)) ;
    flight.velocity(i+1) = Rk4_int(@acceleration_eq,t(i),flight.velocity(i),dt);
    flight.displacement(i+1) = Rk4_int(@velocity_eq,t(i),flight.velocity(i),dt);

end

figure(1)
plot(t, flight.acceleration)
hold on
plot(t, flight.velocity)
plot(t, flight.displacement)
hold off
grid