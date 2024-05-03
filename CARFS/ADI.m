function Z=ADI(alt,data,max_alt)
clc 


% max_alt = 280000;
% Atmospheric_data_280K = xlsread('Atmospheric_data_280K.xlsx');
% alt = Atmospheric_data_280K(:,1);
% mu = Atmospheric_data_280K(:,9);
% data = Atmospheric_data_280K(:,7);
% delta_t = 1/60;

i = 1;
p = 0;
h = 0;
max_alt = max_alt+1;
info = zeros(max_alt, 1);
altitude = zeros(max_alt,1);
n = 1;
m = 2;
    while h <= max_alt        
        if p < 5000           
            p = p+1;

        else
            n = n+1;
            m = m+1;
            p = 1;
        end
        info(i) = data(n)+ (data(m)-data(n))*((alt(n) - h)/(alt(n) - alt(m)));
        altitude(i) = h;
        i = i+1;
        h=h+1;
        if i == max_alt
            break
        else
        end
    end

%Collect Data
T = table(altitude, info);
T= T(1:max_alt,:);
Z = info;
fprintf('DONE\n\n');
% xlswrite('Atmospheric_data_N.xlsx', T,1,'A2');
end




