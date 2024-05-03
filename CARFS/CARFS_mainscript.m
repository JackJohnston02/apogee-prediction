%% Rocket Simulation
% Subsonic and Supersonic 
% Computational Aerodynamic Rocket Flight Simulation (C.A.R.F.S.)
% Programmer(s): Michael Blaser
% copyright © 2018

%% %%%********** UPDATES *********%%%
% Whats up boys, I updated the Averaged_first2inches_nosecone with better
% equations for the ogive series nose cones. 
%
%12/02/2020
%Hello boys in Glasgow, Its been a while since I booted up the program on a
%new Matlab software, it appears Nose Cone Type 3 (Van Karmen) has some
%issues that I am investigating. Please hmu if you have any question my
%email is blaserm129@gmail.com
%
%Must hard code surface area into weather.m
%
%
%
%let me know if there is anything eles not working for me to fix or to bring to my attention.
%% %%%*********MAINSCRIPT*********%%%
clc
fprintf('Hello, Please make sure the necessary information is hardcoded:\n');
fprintf('-Altitude at launch pad\n-Wind Conditions\n-Type of Nose Cone\n');
choice = menu('Have the conditions in the command window been filled in','Yes','No');
if choice == 1
else
    return
end
clc
Stages = 1;
% Altitude at Launch Pad (ASL)
counter = 200;       %ft 
% Wind Conditions
Wind_S = 4 * 1.46667; %ft/s
Wind_S2 = 15* 1.46667;
Wind_S3 = 50* 1.46667;
Wind_LB = 0;
Wind_UB = 10000;
Wind_LB2 = 10000;
Wind_UB2 = 18000;
Wind_LB3 = 18000;
Wind_UB3 = 30000;


%% Type of Nose Cone
Nose_cone_type = 2; % 1 = 3/4 power series, 2 = Ogive, 3 = 1/3 Haack series (von Karmen)

%% ***** ATMOSPHERIC DATA BANK ******

Atmospheric_data_280K = xlsread('Atmospheric_data_65K.xlsx');
alt = Atmospheric_data_280K(:,1);
viscosity = Atmospheric_data_280K(:,9);
density = Atmospheric_data_280K(:,7);    % slugs/cubic_inch
temperature = Atmospheric_data_280K(:,5);
pressure = Atmospheric_data_280K(:,6);   %psf
%% **** TABLE OF CONSTANTS****** 
max_alt= 280000;     % Program max alt is 280000(can be changed if there is further informtion on atm. data)
gamma = 1.4;
R = 1716;           %gas constant

%***** SIMULATION Parameters *****
delta_t = 1/200;  %runs simulation at 400 Hertz 
Re_xcr = 5e5;       % critical value; marks the value of critical Reynolds Number

%% CORRECTION FACTORS ** DO NOT TOUCH **
incomp_corr = 0.8;
sub_corr = 1.0;
trans_corr = 1.0; 
super_corr = .85;

%% linear interpolate for more accurate data
rho = ADI(alt,density,max_alt);                 %slugs/ft^3
mu = ADI(alt,viscosity, max_alt)*10e-6;         %10^-6 slugs/fts
temp = ADI(alt,temperature, max_alt);           %Rankine
static_pressure = ADI(alt,pressure, max_alt);   %lb/ft^2

%% Rocket Menu
choice1 = menu ('Choose from the Library?','Yes','No');
if choice1 == 1
    rocketchoice = menu('Which rocket are you looking at:','1) Senior Design','2) Bullpup','3) Dan Welling MiniBalls', '4)Dans Impulse', '5) Nightman aaHHaaaaa', '6) Embry-Riddle', '7) SLURPL Bicentennial', '8) GU Rocketry LC2');
    if rocketchoice == 1    %Senior Design
        diameter = 6.17/12;
        Aft_diameter = 4.65/12;
        L_nose = 33/12;
        L_body = 101/12;
        Root_chord_length = 19/12;
        Tip_chord_length = 4/12;
        fin_height = 6/12;
        fin_thickness = .125/12;
        length_LE = 16/12;
        num_fins = 3;
        LW = 100.0;
        
%         %SLUBLUE
%         Thrust_Y = [1065.27844000000,1071.36157000000,1081.43225000000,1101.27966000000,1113.06543000000,1145.62646484375,1162.40039062500,1180.64294433594,1200.21179199219,1229.38330078125,1260.37109375000,1280.54382324219,1301.29565429688,1323.73437500000,1343.49230957031,1363.16162109375,1383.50439453125,1403.82678222656,1423.33239746094,1436.31274414063,1455.18029785156,1473.84326171875,1455.93957519531,1419.69262695313,1391.73754882813,1367.16821289063,1330.79663085938,1316.28515625,1285.36779785156,1243.47314453125,1216.31127929688,1177.58007812500,1146.31188964844,1124.51733398438,1108.56909179688,0,0];
%         Thrust_time = [0.0,0.040,0.090,0.190,0.2500,0.42000,0.510,0.610,0.720,0.890,1.08,1.210,1.350,1.510,1.660,1.820,2,2.200,2.4200,2.5900,2.9000,3.52000,3.6000,3.650,3.700000,3.82000000000000,3.94000000000000,4.0000000000 4.13000000000000,4.30000000000000,4.40000000000000,4.52000000000000,4.59000000000000,4.62000000000000,4.63000000000000,5,100];
%         max_time = 100;
%         Taverage = 1199.5;
%         t_burn = 4.63;
%         weight_propellant = 28;
        
%         %SLUBLUE MARK 3
%         Thrust_Y = [933.0, 968.9, 989.2, 1067.8, 1124.0, 1164.5, 1214.0, 1258.9, 1281.4, 1294.9, 1303.9, 1309.5, 1309.5, 1247.7, 1101.6, 1056.6, 989.2, 977.9, 0, 0, 0, 0]; 
%         Thrust_time = [0, 0.25, 0.37, 0.9, 1.33, 1.7, 2.175, 2.7, 3.1, 3.4, 3.7, 4, 4.3, 4.45, 5.05, 5.25, 5.5, 5.57, 6.5, 7, 10, 100];
%         max_time = 100;
%         Taverage = 1157.1;
%         t_burn = 5.65;
%         weight_propellant = 27;
       
%         %SLUE5588
%         Thrust_Y = [994.78, 1008.94, 1072.34, 1151.02, 1180.25, 1212.28, 1281.41, 1303.89, 1333.12, 1371.33, 1395.39, 1410.68, 1416.30, 1406.18, 1346.61, 1290.4, 1244.32, 1200.03, 1135.29, 1115.06, 0, 0, 0, 0];
%         Thrust_time = [0, 0.12, 0.5, 1, 1.2, 1.43, 2, 2.26, 2.5, 2.98, 3.38, 3.75, 4, 4.19, 4.3, 4.52, 4.74, 4.95, 5.225, 5.29, 5.4, 8, 15, 100];
%         max_time = 100;
%         Taverage = 1245.9;
%         t_burn = 5.5;
%         weight_propellant = 27.88;
        
%         %SLU5513
%         Thrust_Y = [1024.91, 1107.64, 1183.73, 1252.19, 1290.63, 1337.620, 1391.01, 1409.56, 1423.49, 1434.280, 1448.22, 1450.47, 1382.8, 1344.02, 1314.01, 1285.69, 1259.16, 1232.63, 1175.530, 1141.02, 239.42, 0. 0, 0, 0];
%         Thrust_time = [ 0, 0.49, 0.97, 1.45, 1.75, 2.165, 2.75, 3.01, 3.25, 3.49, 3.97, 4.21, 4.33, 4.45, 4.57, 4.69, 4.810, 4.930, 5.17, 5.29, 5.41, 5.53, 7, 10, 100];
%         max_time = 100;
%         Taverage = 1263.65;
%         t_burn = 5.53;
%         weight_propellant = 28;

%         SLU5318
%         Thrust_Y = [1024.91, 1107.64, 1183.73, 1252.19, 1290.63, 1337.620, 1391.01, 1409.56, 1423.49, 1434.280, 1448.22, 1450.47, 1382.8, 1344.02, 1314.01, 1285.69, 1259.16, 1232.63, 1175.530, 1141.02, 239.42, 0. 0, 0, 0];
%         Thrust_time = [ 0, 0.49, 0.97, 1.45, 1.75, 2.165, 2.75, 3.01, 3.25, 3.49, 3.97, 4.21, 4.33, 4.45, 4.57, 4.69, 4.810, 4.930, 5.17, 5.29, 5.41, 5.53, 7, 10, 100];
%         max_time = 100;
%         Taverage = 1263.65;
%         t_burn = 5.53;
%         weight_propellant = 28;

          %SLU189 49% O
          Thrust_Y = [1001.53, 1008.05, 1155.97, 1222.960, 1260.51, 1307.04, 1358.75, 1376.28, 1390.9, 1401.91, 1416.30, 1418.10, 1363.69, 1321.88, 1290.86, 1263.43, 1237.58, 1211.95, 1157.32, 1125.40, 752.21, 0, 0, 0, 0];
          Thrust_time = [ 0, 0.49, 0.97, 1.45, 1.75, 2.165, 2.75, 3.01, 3.25, 3.49, 3.97, 4.21, 4.33, 4.45, 4.57, 4.69, 4.810, 4.930, 5.17, 5.29, 5.41, 5.53, 7, 10, 100];
          max_time = 100;
          Taverage = 1242.52;
          t_burn = 5.52;
          weight_propellant = 27.88;
   
fprintf('Rocket diameter (in): %.2f\n',diameter*12);
        fprintf('Rocket Aft Diameter (in): %.2f\n',Aft_diameter*12);
        fprintf('Length of nose cone (in): %.2f\n',L_nose*12);
        fprintf('Length of body (in): %.2f\n',L_body*12);
        fprintf('Root chord length (in): %.2f\n',Root_chord_length*12);
        fprintf('Tip chord length (in): %.2f\n',Tip_chord_length*12);
        fprintf('Fin height (in): %.2f\n',fin_height*12);
        fprintf('Fin thickness (in): %.2f\n',fin_thickness*12);
        fprintf('Length of Leading Edge (in): %.2f\n',length_LE*12);
        fprintf('Total number of fins : %d\n',num_fins);
        fprintf('Total liftoff weight (lbs): %.2f\n\n',LW);
        
        fprintf('Average Thrust (lbs): %.2f\n',Taverage);
        fprintf('Burn Time (seconds): %.2f\n',t_burn);
        fprintf('Propellant Weight (lbs): %.2f\n\n',weight_propellant);

    elseif rocketchoice == 2    %Bull Pup
        diameter = 10/12;
        Aft_diameter = 4/12;
        L_nose = 33.5/12;
        L_body = 75.5/12;
        Root_chord_length = 19.5/12;
        Tip_chord_length = 6/12;
        fin_height = 10.75/12;
        fin_thickness = 0.375/12;
        length_LE = 17.5/12;
        num_fins = 4;
        LW = 109;

        %N5500 Loki Motor: probellant weight: 18.125lbs: burn time:2.4s average
        %thrust: 1236.449
        Thrust_Y =    [1221.9, 1409, 1432.9, 1331.6, 1175.3, 1010.3, 0, 0, 0, 0, 0];
        Thrust_time = [0.0, 0.5, 1.0, 1.6, 2.0, 2.4, 2.6, 3, 4.0, 5, 100];
        
        max_time = 100;
        Taverage = 1236.449;
        t_burn = 2.4;
        weight_propellant = 18.125;
        
        
        fprintf('Rocket diameter (in): %.2f\n',diameter*12);
        fprintf('Rocket Aft Diameter (in): %.2f\n',Aft_diameter*12);
        fprintf('Length of nose cone (in): %.2f\n',L_nose*12);
        fprintf('Length of body (in): %.2f\n',L_body*12);
        fprintf('Root chord length (in): %.2f\n',Root_chord_length*12);
        fprintf('Tip chord length (in): %.2f\n',Tip_chord_length*12);
        fprintf('Fin height (in): %.2f\n',fin_height*12);
        fprintf('Fin thickness (in): %.2f\n',fin_thickness*12);
        fprintf('Length of Leading Edge (in): %.2f\n',length_LE*12);
        fprintf('Total number of fins : %d\n',num_fins);
        fprintf('Total liftoff weight (lbs): %.2f\n\n',LW);
        
        fprintf('Average Thrust (lbs): %.2f\n',Taverage);
        fprintf('Burn Time (seconds): %.2f\n',t_burn);
        fprintf('Propellant Weight (lbs): %.2f\n\n',weight_propellant);
        
    elseif rocketchoice==3  %Dan Welling MiniBalz
        diameter = 2.2/12;
        Aft_diameter = 2/12;
        L_nose = 9/12;
        L_body = 30/12;
        Root_chord_length = 6.3/12;
        Tip_chord_length = 1.7/12;
        fin_height = 3/12;
        fin_thickness = 0.07/12;
        length_LE = 5/12;
        num_fins = 3;
        LW = 6.1;

        %Cessaroni K740 
        Thrust_Y =    [165, 192, 195, 190, 175, 170, 165, 160, 0, 0, 0, 0];%lbs
        Thrust_time = [0.0, 0.05, 0.5, 0.9, 1.5, 1.75, 1.9, 2.125, 2.5, 5, 7, 100];        
        max_time = 100;
        Taverage = 166.38;
        t_burn = 2.5;
        weight_propellant = 2.0;
        
        
        fprintf('Rocket diameter (in): %.2f\n',diameter*12);
        fprintf('Rocket Aft Diameter (in): %.2f\n',Aft_diameter*12);
        fprintf('Length of nose cone (in): %.2f\n',L_nose*12);
        fprintf('Length of body (in): %.2f\n',L_body*12);
        fprintf('Root chord length (in): %.2f\n',Root_chord_length*12);
        fprintf('Tip chord length (in): %.2f\n',Tip_chord_length*12);
        fprintf('Fin height (in): %.2f\n',fin_height*12);
        fprintf('Fin thickness (in): %.2f\n',fin_thickness*12);
        fprintf('Length of Leading Edge (in): %.2f\n',length_LE*12);
        fprintf('Total number of fins : %d\n',num_fins);
        fprintf('Total liftoff weight (lbs): %.2f\n\n',LW);
        
        fprintf('Average Thrust (lbs): %.2f\n',Taverage);
        fprintf('Burn Time (seconds): %.2f\n',t_burn);
        fprintf('Propellant Weight (lbs): %.2f\n\n',weight_propellant);
    
    elseif rocketchoice==4  %Dans Impulse
        diameter = 6.18/12;
        Aft_diameter = 6.18/12;
        L_nose = 34/12;
        L_body = 114/12;
        Root_chord_length = 21.5/12;
        Tip_chord_length = 6/12;
        fin_height = 7/12;
        fin_thickness = 0.225/12;
        length_LE = 15.2/12;
        num_fins = 3;
        LW = 62.7;

        %Cessaroni K740 
        
        Thrust_Y =    [630, 560, 590, 610, 625, 650, 660, 675, 660, 650, 600, 550,530, 0, 0, 0, 0];%lbs
        Thrust_time = [0.0, 0.20, 0.43, 0.60, 0.860, 1.3, 1.72, 1.90, 2.15, 2.5, 3.0, 3.44, 3.88, 4.3, 7, 10, 100];
        
        max_time = 100;
        Taverage = 583.9;
        t_burn = 4.3;
        weight_propellant = 14.6;
        
        
        fprintf('Rocket diameter (in): %.2f\n',diameter*12);
        fprintf('Rocket Aft Diameter (in): %.2f\n',Aft_diameter*12);
        fprintf('Length of nose cone (in): %.2f\n',L_nose*12);
        fprintf('Length of body (in): %.2f\n',L_body*12);
        fprintf('Root chord length (in): %.2f\n',Root_chord_length*12);
        fprintf('Tip chord length (in): %.2f\n',Tip_chord_length*12);
        fprintf('Fin height (in): %.2f\n',fin_height*12);
        fprintf('Fin thickness (in): %.2f\n',fin_thickness*12);
        fprintf('Length of Leading Edge (in): %.2f\n',length_LE*12);
        fprintf('Total number of fins : %d\n',num_fins);
        fprintf('Total liftoff weight (lbs): %.2f\n\n',LW);
        
        fprintf('Average Thrust (lbs): %.2f\n',Taverage);
        fprintf('Burn Time (seconds): %.2f\n',t_burn);
        fprintf('Propellant Weight (lbs): %.2f\n\n',weight_propellant);
        
    elseif rocketchoice == 5    %Nightman rocket (aHHHHHaaaaaa)
        diameter = 4.02/12;
        Aft_diameter = 4.02/12;
        L_nose = 22/12;
        L_body=84/12;
        Root_chord_length = 12/12;
        Tip_chord_length = 3.5/12;
        fin_height = 4/12;
        fin_thickness = 0.1/12;
        length_LE = 8/12;
        num_fins = 3;
        LW = 25.7;
        
        % AeroTech L1300R 
        Thrust_Y = [292.25, 297.87, 299, 337.21,348.45, 339.01, 304.39, 247.29, 245.04, 257.41, 39.57, 0, 0, 0, 0];
        Thrust_time = [0.0, 0.4, 0.5, 1, 1.5, 2, 2.48, 3, 3.125, 3.22, 3.37, 3.44, 7, 10, 100];
        max_time = 100;
        Taverage = 297.44;
        t_burn = 3.44;
        weight_propellant = 5.56;
        
        fprintf('Rocket diameter (in): %.2f\n',diameter*12);
        fprintf('Rocket Aft Diameter (in): %.2f\n',Aft_diameter*12);
        fprintf('Length of nose cone (in): %.2f\n',L_nose*12);
        fprintf('Length of body (in): %.2f\n',L_body*12);
        fprintf('Root chord length (in): %.2f\n',Root_chord_length*12);
        fprintf('Tip chord length (in): %.2f\n',Tip_chord_length*12);
        fprintf('Fin height (in): %.2f\n',fin_height*12);
        fprintf('Fin thickness (in): %.2f\n',fin_thickness*12);
        fprintf('Length of Leading Edge (in): %.2f\n',length_LE*12);
        fprintf('Total number of fins : %d\n',num_fins);
        fprintf('Total liftoff weight (lbs): %.2f\n\n',LW);
        
        fprintf('Average Thrust (lbs): %.2f\n',Taverage);
        fprintf('Burn Time (seconds): %.2f\n',t_burn);
        fprintf('Propellant Weight (lbs): %.2f\n\n',weight_propellant);
        
    elseif rocketchoice == 6    %EMBRYRIDDLE_SPA
        Stages = 1;
        diameter = 5.5/12;
        Aft_diameter = 5.5/12;
        L_nose = 21/12;
        L_body = 97/12;
        Root_chord_length = 8/12;
        Tip_chord_length = 5.2/12;
        fin_height = 6.5/12;
        fin_thickness = .25/12;
        length_LE = 8.7/12;
        num_fins = 4;
        LW = 72.7;
        
        % Cessaroni M1800
        Thrust_Y = [490.427,434.68,468.018,475,452.7020,394.369,391.350,230.52,71.73423, 0, 0, 0, 0];
        Thrust_time = [0, 0.26, 1.312, 1.825, 3.09, 4.48, 4.675, 4.9, 5.38, 5.6, 7, 10, 100];
        max_time = 100;
        Taverage = 401.35;
        t_burn = 5.54;
        weight_propellant = 10.94;
      
%         % Cesaroni N5800
%         Thrust_Y2 = [1500, 1500, 1576, 1570, 1550, 1500, 1430, 1400, 1350, 1200, 1050, 950,700, 400, 200, 0, 0];
%         Thrust_time2 = [0.0, 0.5, 1.0, 1.4, 1.8, 2.0, 2.16, 2.3, 2.6, 2.7, 2.8, 2.88, 3.0, 3.2, 3.4, 3.6, 100];
%         max_time2 = 100;
%         Taverage2 = 1300;
%         t_burn2 = 3.52;
%         weight_propellant2 = 19.88; 
        
        fprintf('Rocket diameter (in): %.2f\n',diameter*12);
        fprintf('Rocket Aft Diameter (in): %.2f\n',Aft_diameter*12);
        fprintf('Length of nose cone (in): %.2f\n',L_nose*12);
        fprintf('Length of body (in): %.2f\n',L_body*12);
        fprintf('Root chord length (in): %.2f\n',Root_chord_length*12);
        fprintf('Tip chord length (in): %.2f\n',Tip_chord_length*12);
        fprintf('Fin height (in): %.2f\n',fin_height*12);
        fprintf('Fin thickness (in): %.2f\n',fin_thickness*12);
        fprintf('Length of Leading Edge (in): %.2f\n',length_LE*12);
        fprintf('Total number of fins : %d\n',num_fins);
        fprintf('Total liftoff weight (lbs): %.2f\n\n',LW);
        
        fprintf('Average Thrust (lbs): %.2f\n',Taverage);
        fprintf('Burn Time (seconds): %.2f\n',t_burn);
        fprintf('Propellant Weight (lbs): %.2f\n\n',weight_propellant);
       
    elseif rocketchoice == 7    %SLURPL Bicentennial
        diameter = 4.5/12;
        Aft_diameter = 4.5/12;
        L_nose = 18/12;
        L_body = 108/12;
        Root_chord_length = 22/12;
        Tip_chord_length = 3/12;
        fin_height = 4.5/12;
        fin_thickness = 0.25/12;
        length_LE = 17.526/12;
        num_fins = 3;
        LW = 69.8;
        
        % SLURPL BTBNH
        Thrust_Y = [1122.95 1177.33 1230.98 1360.84 1435.05 1505.86 1572.79 1674.47 1745.84 1807.40 1846.67 1573.03 1345.53 1056.70 0 0 0 ];%lb
        Thrust_time = [0.00 0.25 0.49 1.09 1.45 2.17 2.77 3.25 3.73 4.09 4.57 5.05 5.53 5.65 7 10 100];
        max_time = 100;
        Taverage = 1465.53;
        t_burn = 5.65;
        weight_propellant = 34.57;
        
        fprintf('Rocket diameter (in): %.2f\n',diameter*12);
        fprintf('Rocket Aft Diameter (in): %.2f\n',Aft_diameter*12);
        fprintf('Length of nose cone (in): %.2f\n',L_nose*12);
        fprintf('Length of body (in): %.2f\n',L_body*12);
        fprintf('Root chord length (in): %.2f\n',Root_chord_length*12);
        fprintf('Tip chord length (in): %.2f\n',Tip_chord_length*12);
        fprintf('Fin height (in): %.2f\n',fin_height*12);
        fprintf('Fin thickness (in): %.2f\n',fin_thickness*12);
        fprintf('Length of Leading Edge (in): %.2f\n',length_LE*12);
        fprintf('Total number of fins : %d\n',num_fins);
        fprintf('Total liftoff weight (lbs): %.2f\n\n',LW);
        
        fprintf('Average Thrust (lbs): %.2f\n',Taverage);
        fprintf('Burn Time (seconds): %.2f\n',t_burn);
        fprintf('Propellant Weight (lbs): %.2f\n\n',weight_propellant);
  
%     elseif rocketchoice == 8    %SLURPL SAC 2017 10k
%         diameter = 6.17/12;
%         Aft_diameter = 6.17/12;
%         L_nose = 30/12;
%         L_body = 94/12;
%         Root_chord_length = 24/12;
%         Tip_chord_length = 2.719/12;
%         fin_height = 6/12;
%         fin_thickness = .25/12;
%         length_LE = 22.8/12;
%         num_fins = 3;
%         LW = 60.3;
%         
%         % SLURPL M
%         Thrust_Y = [415.93 445.26 473.26 516.25 560.36 594.69 611.46 568.57 526.31 464.65 0 0 0 0];%lb
%         Thrust_time = [0.0 0.28 0.55 1 1.54 2.08 2.44 2.98 3.52 3.97 4.15 5 7 100];
%         max_time = 100;
%         Taverage = 470.613;
%         t_burn = 4.15;
%         weight_propellant = 8.89;
%         
%         fprintf('Rocket diameter (in): %.2f\n',diameter*12);
%         fprintf('Rocket Aft Diameter (in): %.2f\n',Aft_diameter*12);
%         fprintf('Length of nose cone (in): %.2f\n',L_nose*12);
%         fprintf('Length of body (in): %.2f\n',L_body*12);
%         fprintf('Root chord length (in): %.2f\n',Root_chord_length*12);
%         fprintf('Tip chord length (in): %.2f\n',Tip_chord_length*12);
%         fprintf('Fin height (in): %.2f\n',fin_height*12);
%         fprintf('Fin thickness (in): %.2f\n',fin_thickness*12);
%         fprintf('Length of Leading Edge (in): %.2f\n',length_LE*12);
%         fprintf('Total number of fins : %d\n',num_fins);
%         fprintf('Total liftoff weight (lbs): %.2f\n\n',LW);
%         
%         fprintf('Average Thrust (lbs): %.2f\n',Taverage);
%         fprintf('Burn Time (seconds): %.2f\n',t_burn);
%         fprintf('Propellant Weight (lbs): %.2f\n\n',weight_propellant);
        
        elseif rocketchoice == 8    %GU Rocketry Launch campaign 2
        
        diameter = 5.27559/12;
        Aft_diameter = 5.27559/12;
        L_nose = 25.5906/12;
        L_body = 72.83465/12;
        Root_chord_length = 5.90551/12;
        Tip_chord_length = 2.95276/12;
        fin_height = 5.90551/12;
        fin_thickness = 0.118/12;
        length_LE = 4.44882/12;
        num_fins = 4;
        LW = 37.4786;
        
        % Cessaroni M795
        Thrust_Y = [0 137.65 344.58 387.12 386.15 346.85 321.48 312.42 308.95 301.00 293.46 285.91 270.67 242.34 208.62 167.03 126.57 95.52 67.38 44.10 37.49 14.71 0 0 ];%lb
        Thrust_time = [0 0.15 0.21 0.25 0.43 0.5 0.62 0.8 1 1.5 2 3 4 5 6 7 8 9 10 11 12 12.7 12.76 100];
        max_time = 100;
        Taverage = 166.3810;
        t_burn = 12.76;
        weight_propellant = 10.7850;
        
        
        
    end
else
    
    choice = menu ('Simulation #1?','Yes', 'No');


    if choice == 2;  
        while (1)
            fprintf('Rocket diameter (in): %.2f\n',diameter*12);
            fprintf('Rocket Aft Diameter (in): %.2f\n',Aft_diameter*12);
            fprintf('Length of nose cone (in): %.2f\n',L_nose*12);
            fprintf('Length of body (in): %.2f\n',L_body*12);
            fprintf('Root chord length (in): %.2f\n',Root_chord_length*12);
            fprintf('Tip chord length (in): %.2f\n',Tip_chord_length*12);
            fprintf('Fin height (in): %.2f\n',fin_height*12);
            fprintf('Fin thickness (in): %.2f\n',fin_thickness*12);
            fprintf('Length of Leading Edge (in): %.2f\n',length_LE*12);
            fprintf('Total number of fins : %d\n',num_fins);
            fprintf('Total liftoff weight (lbs): %.2f\n\n',LW);


            W = input('Are the Rocket Dimensions above correcet? Y/N:','s');
             if W=='Y'|| W=='y'
                 break
             end

            while (1)
                % ***** Requested Variables *****
                display('All units must be in ENGLISH UNITS');
                prompt = 'Rocket diameter (in): ';
                diameter = input(prompt);

                prompt_aft ='Rocket Aft Diameter (in): ';
                Aft_diameter = input(prompt_aft);

                prompt2 = 'Length of nose cone (in): ';
                L_nose = input(prompt2);

                prompt3 = 'Length of body (in): ';
                L_body = input(prompt3);

                prompt4 = 'Root chord length (in): ';
                Root_chord_length = input(prompt4);

                prompt5 = 'Tip chord length (in): ';
                Tip_chord_length = input(prompt5);

                prompt6 = 'Fin height (in): ';
                fin_height = input(prompt6);

                prompt7 = 'Fin thickness (in): ';
                fin_thickness = input(prompt7);

                prompt8 = 'Length of Leading Edge (in): ';
                length_LE = input(prompt8);

                prompt9 = 'Total number of fins: ';
                num_fins = input(prompt9);

                prompt10 = 'Total liftoff weight (lbs): ';
                LW=input(prompt10);   %Liftoff weight, lbs
                clc

                fprintf('Rocket diameter (in): %.2f\n',diameter);
                fprintf('Rocket Aft Diameter (in): %.2f\n',Aft_diameter);
                fprintf('Length of nose cone (in): %.2f\n',L_nose);
                fprintf('Length of body (in): %.2f\n',L_body);
                fprintf('Root chord length (in): %.2f\n',Root_chord_length);
                fprintf('Tip chord length (in): %.2f\n',Tip_chord_length);
                fprintf('Fin height (in): %.2f\n',fin_height);
                fprintf('Fin thickness (in): %.2f\n',fin_thickness);
                fprintf('Length of Leading Edge (in): %.2f\n',length_LE);
                fprintf('Total number of fins: %d\n',num_fins);
                fprintf('Total liftoff weight (lbs): %.2f\n\n',LW);


                diameter = diameter/12;
                Aft_diameter = Aft_diameter/12;
                
                L_nose = L_nose/12;
                L_body = L_body/12;
                Root_chord_length = Root_chord_length/12;
                Tip_chord_length = Tip_chord_length/12;
                fin_height = fin_height/12;
                fin_thickness = fin_thickness/12;
                length_LE = length_LE/12;

                G = input('Are the current Rocket Dimensions Correct? Y/N:','s');
                 if G=='Y'||G=='y'
                     break
                 end
            end
        end
    else
        while (1)
            % ***** Requested Variables *****
            display('All units must be in ENGLISH UNITS');
            prompt = 'Rocket diameter (in): ';
            diameter = input(prompt);

            prompt_aft ='Rocket Aft Diameter (in): ';
            Aft_diameter = input(prompt_aft);

            prompt2 = 'Length of nose cone (in): ';
            L_nose = input(prompt2);

            prompt3 = 'Length of body (in): ';
            L_body = input(prompt3);

            prompt4 = 'Root chord length (in): ';
            Root_chord_length = input(prompt4);

            prompt5 = 'Tip chord length (in): ';
            Tip_chord_length = input(prompt5);

            prompt6 = 'Fin height (in): ';
            fin_height = input(prompt6);

            prompt7 = 'Fin thickness (in): ';
            fin_thickness = input(prompt7);

            prompt8 = 'Length of Leading Edge (in): ';
            length_LE = input(prompt8);

            prompt9 = 'Total number of fins: ';
            num_fins = input(prompt9);

            prompt10 = 'Total liftoff weight (lbs): ';
            LW=input(prompt10);  %Liftoff weight, lbs
            clc

            fprintf('Rocket diameter (in): %.2f\n',diameter);
            fprintf('Rocket Aft Diameter (in): %.2f\n',Aft_diameter);
            fprintf('Length of nose cone (in): %.3f\n',L_nose);
            fprintf('Length of body (in): %.2f\n',L_body);
            fprintf('Root chord length (in): %.2f\n',Root_chord_length);
            fprintf('Tip chord length (in): %.2f\n',Tip_chord_length);
            fprintf('Fin height (in): %.2f\n',fin_height);
            fprintf('Fin thickness (in): %.2f\n',fin_thickness);
            fprintf('Length of Leading Edge (in): %.2f\n',length_LE);
            fprintf('Total number of fins: %d\n',num_fins);
            fprintf('Total liftoff weight: %.2f\n\n',LW);

            diameter = diameter/12;
            Aft_diameter = Aft_diameter/12;
           
            L_nose = L_nose/12;
            L_body = L_body/12;
            Root_chord_length = Root_chord_length/12;
            Tip_chord_length = Tip_chord_length/12;
            fin_height = fin_height/12;
            fin_thickness = fin_thickness/12;
            length_LE = length_LE/12;

            G = input('Are the current Rocket Dimensions Correct? Y/N:','s');
             if G=='Y'||G=='y'
                 break
             end
        end
    end    
    clc
    if choice == 2  
        while (1)
            fprintf('Average Thrust (lbs): %.2f\n',Taverage);
            fprintf('Burn Time (seconds): %.2f\n',t_burn);
            fprintf('Propellant Weight (lbs): %.2f\n\n',weight_propellant);

            W = input('Are the Motor Stats above correcet? Y/N:','s');
             if W=='Y'||W=='y'
                 break
             end

            while (1)
                    % **** Motor Stats **** %%
                    fprintf('\n***** MOTOR STATS *****\n');
                    prompt11 = 'Average Thrust (lbs): ';
                    Taverage =input(prompt11);
                    prompt12 = 'Burn Time (seconds): ';
                    t_burn = input(prompt12);
                    prompt13 = 'Propellant Weight (lbs): ';
                    weight_propellant = input(prompt13);       
                    clc
                    fprintf('Average Thrust (lbs): %.3f\n',Taverage);
                    fprintf('Burn Time (seconds): %.1f\n',t_burn);
                    fprintf('Propellant Weight (lbs): %.3f\n\n',weight_propellant);

                    H = input('Are the current Motor Stats Correct? Y/N:','s');
                    if H=='Y'||H=='y'
                        break
                    end
            end
        end
    else
        while (1)
                % **** Motor Stats **** %%
                fprintf('\n***** MOTOR STATS *****\n');
                fprintf('Have the Thrust and Thrust time vectors been hardcoded?\n')
                choice10 = menu('Please check line-516 and click to continue','Continue');
                if choice10 == 1
                else
                    return
                end
                prompt11 = 'Average Thrust (lbs): ';
                Taverage =input(prompt11);
                prompt12 = 'Burn Time (seconds): ';
                t_burn = input(prompt12);
                prompt13 = 'Propellant Weight (lbs): ';
                weight_propellant = input(prompt13);       
                clc
                fprintf('Average Thrust (lbs): %.3f\n',Taverage);
                fprintf('Burn Time (seconds): %.1f\n',t_burn);
                fprintf('Propellant Weight (lbs): %.3f\n\n',weight_propellant);

                H = input('Are the current Motor Stats Correct? Y/N:','s');
                if H=='Y'||H=='y'
                    break
                end
        end
    end

    Thrust_Y = [1065.27844000000,1071.36157000000,1081.43225000000,1101.27966000000,1113.06543000000,1145.62646484375,1162.40039062500,1180.64294433594,1200.21179199219,1229.38330078125,1260.37109375000,1280.54382324219,1301.29565429688,1323.73437500000,1343.49230957031,1363.16162109375,1383.50439453125,1403.82678222656,1423.33239746094,1436.31274414063,1455.18029785156,1473.84326171875,1455.93957519531,1419.69262695313,1391.73754882813,1367.16821289063,1330.79663085938,1316.28515625,1285.36779785156,1243.47314453125,1216.31127929688,1177.58007812500,1146.31188964844,1124.51733398438,1108.56909179688,0,0];
    Thrust_time = [0 ,0.040,0.090,0.190,0.2500,0.42000,0.510,0.610,0.720,0.890,1.08,1.210,1.350,1.510,1.660,1.820,2,2.200,2.4200,2.5900,2.9000,3.52000,3.6000,3.650,3.700000,3.82000000000000,3.94000000000000,4.0000000000 4.13000000000000,4.30000000000000,4.40000000000000,4.52000000000000,4.59000000000000,4.62000000000000,4.63000000000000,5,100];
    max_time = 100;
end
Radius = diameter/2;
prompt_laoa = 'Launch Angle of Attack? (deg): ';
LAOA = 90 - input(prompt_laoa);
LAOA = LAOA*pi/180;
mass = LW/32.17;    %slugs
clc

tic % timer start 
%% ********CALCULATION TIME******* %%
% %** Graphing **
% % NOSE Cone
% figure(1)
% x = 0:0.01:(L_nose);
% nose_cone_angle = (atan((Radius)/L_nose)); %angle of deflection 
% nose_cone_angle_deg = nose_cone_angle*180/pi;
% y = (Radius/L_nose)*x;
% y2 = (-1*Radius/L_nose)*x;
% plot(x,y,'b',x,y2,'b');
% hold on
% %Body Tube
% plot([L_nose (L_nose+L_body)],[Radius Radius],'b');
% hold on
% plot([L_nose (L_nose+L_body)],[-Radius -Radius],'b');
% grid on
% 
% axis([0 10 -4 4]);
% 
% % Fins
% hold on
% x2 = (L_body+L_nose-Root_chord_length):0.01:(sqrt(length_LE^2-fin_height^2));
% y3 = length_LE*(x-(L_body+L_nose-Root_chord_length))+Radius;
% plot(x2,y3,'b');

%** VARIABLES FOR FLIGHT **
Swet_nose = SA_nose(Radius,L_nose,Nose_cone_type);
Sref_nose = (pi*Radius^2);
Swet_body = pi*diameter*L_body; % Total wetted area (area's exposed)
Sref_body = 0;                  % Total frontal area (reference area)
Boat_tail = (Aft_diameter/diameter)^3;
Swet_fin = ((((Root_chord_length+Tip_chord_length)/2)*fin_height)); %%%%%%% THIS DOES NOT FOLLOW MATH RULES----DEALWITHIT
Sref_fin = fin_height*fin_thickness;
Sref_fin = Sref_fin*num_fins;
S = Sref_nose + Sref_fin;

wing_sweep = acos(fin_height/length_LE);
sweep_length = fin_height*tan(wing_sweep);

Average_angle_2inch = Averaged_first2inches_nosecone(L_nose,Radius,delta_t,Nose_cone_type);

fin_sweep_qrtrchord = atan((sweep_length + .25*Tip_chord_length -.25*Root_chord_length)/fin_height);

%****** STAND BY LAUNCH Re & Cf ****
Re_nose = 0; 
Cf_nose = 0;
Re_body = 0;
Cf_body = 0;
C_bar = (2/3)*(Root_chord_length+Tip_chord_length-(Root_chord_length*Tip_chord_length)/(Root_chord_length+Tip_chord_length)); %m.g.c. for the fins

%Average propellant mass ejection, slugs/sec
mdot = (weight_propellant/32.17)/t_burn;

%Table zeroing
Moutput = zeros(1+max_time/delta_t,9);
JoeData = zeros(1+max_time/delta_t,7);
Density = zeros(1+max_time/delta_t,5);
Reynolds = zeros(10000, 4);
CD_supersonic_data = zeros(10000,5);
thrust = zeros(round(t_burn/delta_t),1);
Motor_time = zeros(round(t_burn/delta_t),1);

% Simulate rocket flight from time = zero
a = 0.0;        %Speed of sound
q_bar = 0.0;    %dynamic pressu
Mach = 0.0;     %Mach Number
Vdot = 0.0;     %Acceleration
altitude = 0.0; %altitude
time = 0.0;
time_x = 0.0;
time_x2 = 0.0;
time_x3 = 0.0;
time_x_decent = 0.0;
time_x_decent2 = 0.0;
time_x_decent3 = 0.0;
time_exposed = 0.0;
time_exposed2 = 0.0;
time_exposed3 = 0.0;
ascending = true;       %we are moving up after ignition
row = 1;
row1 = 1;
V_fps = 0.0;            %Initial speed
distance_horz = 0.0;
distance_horz2 = 0.0;
distance_horz3 = 0.0;
iterations = 0.0;
attachedshock = 0;
kip1 = 1;
Cd_p = zeros(1000,1);
thrust_time = 1;
hitsuper = 0;

Wind_F = 0.0;
Wind_F2 = 0.0;
Wind_F3 = 0.0;



%  Drag_pressure = 0;
CD = 0;
% Run in a loop to simulate rocket flight from time = zero
while (ascending)
    CD = real(CD);
    P = static_pressure(counter);
    if altitude < 0
        altitude = abs(round(altitude));
    else
    end
    
    if attachedshock == 0;
        % Compute Drag Force using q_bar from previous step
        Drag = CD*q_bar*S;

        % Determine Thrust Force (interpolating between points)
        Thrust = interp1(Thrust_time, Thrust_Y, time);
        
                 
        % Compute Acceleration
        Vdot = (Thrust - Drag - LW) / mass;
    else
        
        % Determine Thrust Force (interpolating between points)
        Thrust = interp1(Thrust_time, Thrust_Y, time);
        
        % Compute Acceleration
        Vdot = (Thrust - Drag - LW) / mass;
    end
    
    if Thrust > 0;
        thrust(thrust_time) = Thrust;
        Motor_time(thrust_time) = time;
        thrust_time = thrust_time + 1;
    else
    end
    
    % Integrat acceleration to update velocity
    if (altitude < 0.01 && Vdot < 0.0)
        V_fps = 0.0;
    else
        V_fps = V_fps + Vdot*delta_t;
    end
    
    if (V_fps > 0)
        
        % Speed O Sound
        a = sqrt(gamma*R*temp(counter));
        
        Mach = round((V_fps/a),2);
        
        
        if Mach < 0.3 % incompressible Subsonic       
            P = static_pressure(counter);
            Re_body_nose = (rho(counter) * V_fps * (L_body+L_nose))/ mu(counter);
            Re_fin = (rho(counter) * V_fps * C_bar)/ mu(counter);
            
            if Re_body_nose < Re_xcr        %laminar flow          
                Cf_body_nose = 1.328/(sqrt(Re_body_nose));          
                Cf_fin = (1.328/(sqrt(Re_fin)));              
            else %Turbulent flow                
                Cf_body_nose = 0.074/(Re_body_nose^0.2);           
                Cf_fin = (0.074/(Re_fin^0.2));               
            end                       
            
            %Fin calculations only dealing with friction
            Z = (2-Mach^2)*cos(fin_sweep_qrtrchord)/(sqrt(1-Mach^2*cos(fin_sweep_qrtrchord)*cos(fin_sweep_qrtrchord)));
            K_fin = (1+Z*(fin_thickness/C_bar)+100*(fin_thickness/C_bar)^4);
           
            K_body_nose = ((1.5/((((L_body+L_nose)/(diameter))-1)^1.2))+1);
                       
            Cd_BT = Cf_body_nose*(1+(60/(((L_body+L_nose)/diameter)^3))+0.0025*((L_body+L_nose)/diameter))*((Swet_body+Swet_nose)/(Sref_body+Sref_nose));
            Cd_fin = ((K_fin * Cf_fin * Swet_fin)/ Sref_fin);
            Cd_base = 0.029*Boat_tail/sqrt(Cd_BT);      %Pressure Drag (Base Drag)           
            
            CD = incomp_corr*(Cd_BT+Cd_fin+Cd_base);
            
            %  Compute Dynamic Pressure
            q_bar = 0.5 * rho(counter) * V_fps^2;
            
            attachedshock = 0;
            
            Rho_nose = rho(counter);
            Rho_body = rho(counter);
            Rho_fin = rho(counter);
            
            Re_nose = Re_body_nose;
            Re_body = Re_body_nose;
        
        elseif (0.3 <= Mach) && (Mach <.8)   %Subsonic
%           pg.199 INTRO TO FLIGHT: ANDERSON
            
            t0 = temp(counter)*(1+((gamma-1)/2)*Mach^2);
            p0 = static_pressure(counter)*(1+((gamma-1)/2)*Mach^2)^(gamma/(gamma-1));   %total pressure            
            rho0 = rho(counter)*(1+((gamma-1)/2)*Mach^2)^(1/(gamma-1));                 %Anderson's book
 
            
            Re_body_nose = (rho0 * V_fps * (L_body+L_nose))/ mu(counter);
            Re_fin = (rho0 * V_fps * C_bar)/ mu(counter);
            
            
            if Re_body_nose < Re_xcr    %laminar        
                Cf_body_nose = 1.328/(sqrt(Re_body_nose));          
                Cf_fin = (1.328/(sqrt(Re_fin)));
            else
                Cf_body_nose = 0.074/(Re_body_nose^0.2);     %Anderson's book      
                Cf_fin = (0.074/(Re_fin^0.2));
                
                %Compressibility corrections subsonic speeds
                % openrocket.sourceforge.net/techdoc.pdf
                
                Cfc_body_nose = Cf_body_nose*(1-0.1*Mach^2);
                Cfc_fin = Cf_fin*(1-0.1*Mach^2);
            end
           
            %Fin calculations only dealing with friction
            Z = (2-Mach^2)*cos(fin_sweep_qrtrchord)/(sqrt(1-Mach^2*cos(fin_sweep_qrtrchord)*cos(fin_sweep_qrtrchord)));
            K_fin = (1+Z*(fin_thickness/C_bar)+100*(fin_thickness/C_bar)^4);
           
            K_body_nose = ((1.5/((((L_body+L_nose)/(diameter))-1)^1.2))+1);
            
                       
            Cd_fin = ((K_fin * Cfc_fin * Swet_fin)/ Sref_fin);
            Cd_BT = Cfc_body_nose*(1+(60/(((L_body+L_nose)/diameter)^3))+0.0025*((L_body+L_nose)/diameter))*((Swet_body+Swet_nose)/(Sref_body+Sref_nose));
            Cd_p = 0.029*(Boat_tail)/sqrt(Cd_BT);   %Pressure Drag (Base Drag)
            
            CD = sub_corr*(Cd_BT + Cd_fin + Cd_p);
     
            %  Compute Dynamic Pressure
            q_bar = 0.5 * rho(counter) * V_fps^2;
            
            Rho_rocket = rho(counter);
            attachedshock = 0;
                        
            Re_nose = Re_body_nose;
            Re_body = Re_body_nose;
           
            Rho_nose = rho0;
            Rho_body = rho0;
            Rho_fin = rho0;
        else    % Sonic
           if Mach >0.8 && Mach < 1.2       %Transonic ~ Mach == 1.0
                Mnormal = sqrt(((gamma-1)*Mach^2+2)/(2*gamma*Mach^2-(gamma-1)));
                t1 = temp(counter)*((2*gamma*Mach^2 - (gamma-1))*((gamma-1)*Mach^2 +2)/(((gamma+1)^2)*Mach^2));
                rho1 = rho(counter)*((((gamma+1)*Mach^2)/((gamma-1)*Mach^2 + 2)))^-1;
                
                Re_body_nose = (rho1 * V_fps * (L_body+L_nose))/ mu(counter);
                Re_fin = (rho1 * V_fps * C_bar)/ mu(counter);

%                 Cf_body_nose = 1.328/(sqrt(Re_body_nose));          
%                 Cf_fin = (1.328/(sqrt(Re_fin)));
                
                if Re_body_nose < Re_xcr        
                    Cf_body_nose = 1.328/(sqrt(Re_body_nose));          
                    Cf_fin = (1.328/(sqrt(Re_fin)));
                else
%                     Cf_body_nose = 1/((1.5*log(Re_body_nose)-5.6)^2);
%                     Cf_fin = 1/((1.5*log(Re_fin)-5.6)^2);
                    %Cf_body_nose = 0.455/((log(Re_body_nose))^2.58);  
                    %Cf_fin = 0.455/((log(Re_fin))^2.58);
                    Cf_body_nose = 0.074/(Re_body_nose^0.2);           
                    Cf_fin = (0.074/(Re_fin^0.2));
                end

                Cfc_body_nose = Cf_body_nose*(1-0.1*Mach^2);
                Cfc_fin = Cf_fin*(1-0.1*Mach^2);
                %Fin calculations only dealing with friction
                Z = (2-Mach^2)*cos(fin_sweep_qrtrchord)/(sqrt(1-Mach^2*cos(fin_sweep_qrtrchord)*cos(fin_sweep_qrtrchord)));
                K_fin = (1+Z*(fin_thickness/C_bar)+100*(fin_thickness/C_bar)^4);

                K_body_nose = ((1.5/((((L_body+L_nose)/(diameter))-1)^1.2))+1);


    %           Cd_body_nose = (K_body_nose * Cfc_body_nose * (Swet_body+Swet_nose))/ Sref_nose;
                Cd_BT = Cfc_body_nose*(1+(60/(((L_body+L_nose)/diameter)^3))+0.0025*((L_body+L_nose)/diameter))*((Swet_body+Swet_nose)/(Sref_body+Sref_nose));
                Cd_fin = ((K_fin * Cfc_fin * Swet_fin)/ Sref_fin);
                Cd_base = 0.029*(Boat_tail)/sqrt(Cd_BT); %Pressure drag (Base Drag)

                CD = trans_corr*(Cd_BT+Cd_fin+Cd_base);
               
                %  Compute Dynamic Pressure
                q_bar = 0.5 * rho1 * V_fps^2;
                
                attachedshock = 0;
                
                P_nose = static_pressure(counter);
                P_body = static_pressure(counter);
                P_fin = static_pressure(counter);
                
                Rho_nose = rho1;
                Rho_body = rho1;
                Rho_fin = rho1;

                
                Re_nose = Re_body_nose;
                Re_body = Re_body_nose;

                T_nose = t1;
                T_body = t1;
                T_fin = t1;
                
                Cd_nose = Cd_BT;
           else % Supersonic
                nose_cone_angle_deg = Average_angle_2inch;
                Theta_max = round((max_mach_deflect(Mach,gamma)),2);
                
                if Theta_max< nose_cone_angle_deg  %Determining if bow shock present
                    Mnormal = sqrt(((gamma-1)*Mach^2+2)/(2*gamma*Mach^2-(gamma-1)));
                    t1 = temp(counter)*((2*gamma*Mach^2 - (gamma-1))*((gamma-1)*Mach^2 +2)/(((gamma+1)^2)*Mach^2));
                    rho0 = rho(counter)*((((gamma+1)*Mach^2)/((gamma-1)*Mach^2 + 2)))^-1;
                                      
                    Re_body_nose = (rho0 * V_fps * (L_body+L_nose))/ (mu(counter)*((t1/temp(counter))^1.5)*((temp(counter)+198.72)/(t1+198.72)));
                    Re_fin = (rho0 * V_fps * C_bar)/ (mu(counter)*((t1/temp(counter))^1.5)*((temp(counter)+198.72)/(t1+198.72)));

                    if Re_body_nose < Re_xcr             
                        Cf_body_nose = 1.328/(sqrt(Re_body_nose));          
                        Cf_fin = (1.328/(sqrt(Re_fin)));
                    else                   
                        Cf_body_nose = 0.074/(Re_body_nose^0.2);           
                        Cf_fin = (0.074/(Re_fin^0.2));                     
                    end
                     
                    %Fin calculations only dealing with friction
                    Z = (2-Mnormal^2)*cos(fin_sweep_qrtrchord)/(sqrt(1-Mnormal^2*cos(fin_sweep_qrtrchord)*cos(fin_sweep_qrtrchord)));
                    K_fin = (1+Z*(fin_thickness/C_bar)+100*(fin_thickness/C_bar)^4);

                    K_body_nose = ((1.5/((((L_body+L_nose)/(diameter))-1)^1.2))+1);

                    %Compressibility corrections subsonic speeds
                    Cfc_body_nose = Cf_body_nose*(1-0.1*Mnormal^2);
                    Cfc_fin = Cf_fin*(1-0.1*Mnormal^2);
                    
                    Cd_BT = Cfc_body_nose*(1+(60/(((L_body+L_nose)/diameter)^3))+0.0025*((L_body+L_nose)/diameter))*((Swet_body+Swet_nose)/(Sref_body+Sref_nose));
                    Cd_fin = ((K_fin * Cfc_fin * Swet_fin)/ Sref_fin);
                    Cd_base = 0.029*Boat_tail/sqrt(Cd_BT); %Pressure Drag
                                        
                    CD = super_corr*(Cd_BT+Cd_fin+Cd_base);
                    
                    %Compute Dynamic Pressure
                    q_bar = 0.5 * rho0 * V_fps^2;
                    
                    P_nose = static_pressure(counter);
                    P_body = static_pressure(counter);
                    P_fin = static_pressure(counter);
                    
                    Rho_nose = rho0;
                    Rho_body = rho0;
                    Rho_fin = rho0;
                    
                    Re_nose = Re_body_nose;
                    Re_body = Re_body_nose;
                   
                    T_nose = t1;
                    T_body = t1;
                    T_fin = t1;
                    
                    Cd_nose = Cd_BT;
                    attachedshock = 0;
                else    
                    %% Nose Cone simulations
                    if hitsuper == 0 %Confirming attached shock wave
                        hitsuper = 1;
                    elseif hitsuper >= 1
                        hitsuper = 2;
                    else
                    end
                    [M3,Cd_nose,T_body,P_body,Rho_body, Re_nose, Rho_nose] = Nose_cone_simMark4(delta_t, hitsuper, L_nose, gamma, Mach, Radius,rho(counter),static_pressure(counter),temp(counter),mu(counter),Swet_nose,Sref_nose,Nose_cone_type); 
                    
                    %% Drag calculations for supersonic flow
                    %*** Needed variables ***%
                    Rho_fin = Rho_nose*(((gamma+1)*Mach^2)/((gamma-1)*Mach^2+2));
                    P_fin = static_pressure(counter)*(1+((2*gamma/(gamma+1))*(Mach^2-1)));
                    T_fin = temp(counter)*((1 +((2*gamma)/(gamma+1))*(Mach^2-1))*(((2+(gamma-1)*Mach^2))/((gamma+1)*Mach^2)));

                    a_body = sqrt(gamma*R*T_body);
                    Re_body = (Rho_body * M3*a_body * L_body)/(mu(counter)*((T_body/temp(counter))^1.5)*((temp(counter)+198.72)/(T_body+198.72)));

                    a_fin = sqrt(gamma*R*T_fin);
                    Re_fin = (Rho_fin * Mach*a_fin * C_bar)/(mu(counter)*((T_fin/temp(counter))^1.5)*((temp(counter)+198.72)/(T_fin+198.72)));                    
                    
                    %Fin calculations only dealing with friction
                    Z = (2-Mach^2)*cos(fin_sweep_qrtrchord)/(sqrt(1-Mach^2*cos(fin_sweep_qrtrchord)*cos(fin_sweep_qrtrchord)));
                    K_fin = (1+Z*(fin_thickness/C_bar)+100*(fin_thickness/C_bar)^4);
                    
                    %Body calculations only dealing with friction
                    K_body_nose = ((1.5/((((L_body+L_nose)/(diameter))-1)^1.2))+1);
                    
                    %Coefficient of friction
                    Cf_body = 0.074/(Re_body^0.2);
                    Cf_fin = 0.074/(Re_fin^0.2);                   
                    
                    Cd_BT = Cf_body*(1+(60/((L_body/diameter)^3))+0.0025*(L_body/diameter))*Swet_body/Sref_nose;         
                    Cd_base = 0.029*Boat_tail/sqrt(Cd_BT);%Pressure Drag
                    Cd_fin = K_fin*Cf_fin*Swet_fin/Sref_fin;
                                
                    CD = super_corr*real(Cd_BT + Cd_base + Cd_fin + Cd_nose);
                                     
                    q_bar = 0.5 * Rho_body * (Mach*a_body)^2; 
                                                                    
                    Drag = CD*q_bar*S;
                    attachedshock = 1;
                end
            end
        end
    end
    
    %% Compiling and Weather application
  % **** EFFECTS OF WEATHER ***** 
  if (Wind_LB < altitude)&& (altitude < Wind_UB)
      [Wind_F, time_exposed] = Weather(rho(counter),Wind_S,delta_t,time_exposed);
  elseif (Wind_LB2 < altitude)&& (altitude < Wind_UB2)
      [Wind_F2, time_exposed2] = Weather(rho(counter),Wind_S2,delta_t,time_exposed2);
  elseif (Wind_LB3 < altitude)&& (altitude < Wind_UB3)
      [Wind_F3, time_exposed3] = Weather(rho(counter),Wind_S3,delta_t,time_exposed3);
  end
  
  if (Wind_UB > altitude) && (altitude > Wind_LB)
      time_x = time_x+delta_t;
  elseif (Wind_UB2 > altitude) && (altitude > Wind_LB2)
      time_x2 = time_x2+delta_t;
  elseif (Wind_UB3 > altitude) && (altitude > Wind_LB3)
      time_x3 = time_x3+delta_t;
  else
  end
  
  
  %  Integrate velocity to update altitude
  altitude = altitude + V_fps*delta_t*sin(LAOA)-.5*32.2*delta_t^2;
  if altitude < 0
      altitude =0;
  else
  end
  
  %fprintf('Altitude: %.2f\n',altitude);

  distance_horz = distance_horz + V_fps*delta_t*cos(LAOA);
  
  %  Update mass based on average propellant loss
  mass = mass - mdot*delta_t*(Thrust/Taverage);

  %  Update time
  time = time + delta_t;  
  
   if Mach>= 1.0 
  CD_supersonic_data(row1, :) = [Mach Cd_BT Cd_nose Cd_base Cd_fin ];
     row1 = row1+1;
   else
   end
  
  row = row + 1;
  
  %  Check for doneness
  if ((altitude > 1.0 && V_fps < 0.0) || (time > max_time))
    ascending = false;
  else
    ascending = true;
  end
  
  kip1 = kip1+1;
  
  
if round(toc) == 10     %Program Timer to show programs runtime
    clc
    fprintf('Time passed: 10 sec\n');
elseif round(toc) == 60
    clc
    fprintf('Time passed: 1 min\n');
    fprintf('Altitude: %.2f\n',altitude);
    fprintf('Mach: %.2f\n', Mach);
elseif round(toc) == 120
    clc
    fprintf('Time passed: 2 min\n');
    fprintf('Altitude: %.2f\n',altitude);
    fprintf('Mach: %.2f\n', Mach);
elseif round(toc) == 300
    clc
    fprintf('Time passed: 5 min\n');
    fprintf('Altitude: %.2f\n',altitude);
    fprintf('Mach: %.2f\n', Mach);
elseif round(toc) == 420
    clc
    fprintf('Time passed: 7 min\n');
    fprintf('Altitude: %.2f\n',altitude);
    fprintf('Mach: %.2f\n', Mach);
elseif round(toc) == 600
    clc
    fprintf('Time passed: 10 min\n');
    fprintf('Altitude: %.2f\n',altitude);
    fprintf('Mach: %.2f\n', Mach);
    
elseif round(toc) == 720
    clc
    fprintf('Time passed: 12 min\n');
    fprintf('Altitude: %.2f\n',altitude);
    fprintf('Mach: %.2f\n', Mach);
elseif round(toc) == 900
    clc
    fprintf('Time passed: 15 min\n');
    fprintf('Altitude: %.2f\n',altitude);
    fprintf('Mach: %.2f\n', Mach);
elseif round(toc) == 1200
    clc
    fprintf('Time passed: 20 min\n');
    fprintf('Atlitude: %f\n', altitude);
    fprintf('Mach: %.2f\n', Mach);
else
end

 %  Collect time history data
  Moutput(row , :) = [time altitude mass Thrust V_fps a Mach CD Drag];
  JoeData(row, :) = [time altitude V_fps Vdot q_bar Thrust mass];
  %Density(row, : ) = [time Mach Rho_nose Rho_body Rho_fin];
 
 %  Reynolds(row , :) = [Mach Re_nose Re_body Re_fin];

if (altitude> 1000) && (V_fps < 100)
    clc
    fprintf('Almost Done\n');
elseif (altitude> 1000) && (V_fps < 50)
    clc
    fprintf('Done\n');
else
end

  
  if altitude < counter
  else
      counter = round(altitude);
  end
  
  iterations = iterations +1;
end
altitude_apogee = altitude; 

%% Decent Calculations
time_exposed_decent = 0.0;
time_exposed_decent2 = 0.0;
time_exposed_decent3 = 0.0;

while altitude_apogee > 0
    if altitude_apogee > 1500
        Vy = 100; %(fps) Decent rate under drogue parachute
    else
        Vy = 20;
    end
    
    if (Wind_LB < altitude_apogee)&& (altitude_apogee <= Wind_UB)
        [Wind_F,time_exposed_decent] = Weather(rho(counter),Wind_S,delta_t,time_exposed_decent);
    elseif (Wind_LB2 < altitude_apogee)&& (altitude_apogee <= Wind_UB2)
         [Wind_F2,time_exposed_decent2] = Weather(rho(counter),Wind_S2,delta_t,time_exposed_decent2);
    elseif (Wind_LB3 < altitude_apogee) && (altitude_apogee <= Wind_UB3)
         [Wind_F3,time_exposed_decent3] = Weather(rho(counter),Wind_S3,delta_t,time_exposed_decent3);
    else
    end
    
    
    
    if (Wind_UB3 > altitude_apogee) &&  (Wind_LB3 < altitude_apogee)
        time_x_decent3 = time_x_decent3+delta_t;
    elseif (Wind_UB2 > altitude_apogee) &&  (Wind_LB2 < altitude_apogee)
        time_x_decent2 = time_x_decent2+delta_t;
    elseif (Wind_UB > altitude_apogee) &&  (Wind_LB < altitude_apogee)
        time_x_decent = time_x_decent+delta_t;        
    end
    
    altitude_apogee = altitude_apogee -Vy*delta_t - .5*32.2*delta_t^2;
    
end


%% **** After Simulation Data Organization **** %%
Moutput(:,3)= Moutput(:,3)*32.17;
Velocity_Horz = (Wind_F/(LW-weight_propellant))*time_exposed_decent;
Velocity_Horz2 = (Wind_F2/(LW-weight_propellant))*time_exposed_decent2;
Velocity_Horz3 = (Wind_F3/(LW-weight_propellant))*time_exposed_decent3;

distance_horz1 = round(Velocity_Horz * time_x + distance_horz,2);
distance_horz2 = round(Velocity_Horz2 * time_x2 + distance_horz2,2);
distance_horz3 = round(Velocity_Horz3 * time_x3 + distance_horz3,2);
Total_distance_horz_ascending = distance_horz1 + distance_horz2 + distance_horz3;

Velocity_Horz_decent3 = (Wind_F3/ (LW-weight_propellant))*time_exposed_decent3;
Velocity_Horz_decent2 = (Wind_F2/ (LW-weight_propellant))*time_exposed_decent2;
Velocity_Horz_decent = (Wind_F/ (LW-weight_propellant))*time_exposed_decent;


Distance_horz_decent = round(Velocity_Horz_decent3 * time_x_decent3,2);
Distance_horz_decent2 = round(Velocity_Horz_decent2 * time_x_decent2 + Distance_horz_decent,2);
Distance_horz_decent3 = round(Velocity_Horz_decent * time_x_decent + Distance_horz_decent2,2);


Total_horz_dist = Total_distance_horz_ascending + Distance_horz_decent3; 
% Affected_alt = sqrt(((altitude-Wind_LB)^2)-Distance_Horz^2)+Wind_LB;

Moutput = Moutput(1:(row-1),:);
JoeData = JoeData(1:(row-1),:);

%Reynolds = Reynolds(1:(row-1),:);

% figure(2), clf, subplot(211), plot(Moutput(:,1), Moutput(:,2)), grid on
% xlabel('Time (sec)'), ylabel('Altitude (feet)'), title('Altitude')
% subplot(212), plot(Moutput(:,1), Moutput(:,3)), grid on
% xlabel('Time (sec)'), ylabel('Velocity (fps)'), title('Velocity')

figure(2), clf,plot(Moutput(:,7), Moutput(:,8)), grid on
xlabel('Mach Number)'), ylabel('CD'), title('CD vs Mach')

figure(3), clf, subplot(211), plot(Motor_time, thrust), grid on
xlabel('Time (sec)'), ylabel('Thrust (pounds)'), title('Thrust')
subplot(212), plot(Moutput(:,1), Moutput(:,3)), grid on
xlabel('Time (sec)'), ylabel('Mass (lbs)'), title('Mass')

figure(4), clf, subplot(211), plot(Moutput(:,1), Moutput(:,8)), grid on
xlabel('Time (sec)'), ylabel('CD'), title ('CD vs time')
subplot(212), plot(Moutput(:,1), Moutput(:,9)), grid on
xlabel('Time (sec)'), ylabel('Drag (pounds)'), title ('Drag')

figure(5), clf, subplot(311), plot(Moutput(:,1), Moutput(:,2)),grid on 
xlabel('Time (sec)'), ylabel('Altitude (feet)'), title ('Altitude')
subplot(312), plot(Moutput(:,1), Moutput(:,7)), grid on
xlabel('Time (sec)'), ylabel('Mach'), title ('Mach vs time')
subplot(313), plot(Moutput(:,1), Moutput(:,9)), grid on
xlabel('Time (sec)'), ylabel('Drag (pounds)'), title ('Drag vs time')


% output data point to file in EXCEL format
d = {'Time' , 'Altitude', 'V_fps','a','Mach', 'mass', 'Thrust', 'CD', 'Drag'};
xlswrite('RunSim_SeniorDesign', d, 1, 'A1');        %The files output name
xlswrite('RunSim_SeniorDesign', Moutput, 1, 'A2');
done = datestr(now);

d1 = {'Time', 'Altitude', 'V_fps','V_dot', 'dyn_pressure', 'Thrust', 'Mass'};
xlswrite('JoeData', d1, 1, 'A1');                   %The Files output name
xlswrite('JoeData', JoeData, 1, 'A2');
done1 = datestr(now);

fprintf('\nDone\n');
fprintf('\nMax altitude is %g feet\n', (max(Moutput(:,2))));
%fprintf('Corrected Max altitude: %g feet\n\n', Affected_alt);
%fprintf('Distance from Launch Pad: %g feet\n', Distance_Horz);
fprintf('Max Mach = %.2f\n', max(Moutput(:,7)));
fprintf('\nProgram Run Time: %.2f sec.\n',toc);

load handel %These next lines are meant to provide humor when bored
%sound(y,Fs);
    