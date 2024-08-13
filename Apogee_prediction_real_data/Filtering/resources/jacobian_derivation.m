syms g_0 R_e x_2 x_4 x_1 T_0 M R P_0 L

rho = (P_0 * M) / (R * T_0) * (1 - (L * x_1)/(T_0))^((-M * g_0 * ((R_e)/(R_e + x_1))^2)/(R * L) - 1);


f_3 = g_0 * (R_e / (R_e + x_1))^2 - (x_2^2) / (2 * x_4) * rho;


df_3 = diff(f_3, x_1);

disp(df_3)
