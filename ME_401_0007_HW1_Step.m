rho = .0023769; % (slugs/ft^3) air density at sea level and 60 deg F
da = deg2rad(25);

% navion %
s_n = 184; % (ft^2) wing surface area 
b_n = 33.4; % (ft) wing span 
c_lp_n = -.410; % roll damping stability derivative
w_n_lbs = 2750; % (lbs) weight
w_n_slugs = w_n_lbs / 32.174 ; % (slugs) weight
I_x_n = 1048; % (slugs/ft^2) mass moment of inertia x
I_x_wingtips_n =7003; %I_x_n +(.125 *w_n_slugs)*((b_n/2)^2); % (slugs/ft^2) mass moment of inertia x when fuel is in the wingtips
c_lda_n = -.134; % aileron effectiveness control derivative
c_lda_right_n = -.067;

% F-104A %
s_f = 196.1; % (ft^2) wing surface area 
b_f = 21.94; % (ft) wing span 
c_lp_f = -.285; % roll damping stability derivative 
w_f_lbs = 16300; % (lbs) weight
w_f_slugs = w_f_lbs / 32.174 ; % (slugs) weight
I_x_f = 3549; % (slugs/ft^2) mass moment of inertia x
I_x_wingtips_f =18779; %I_x_f +(.125 *w_f_slugs)*((b_f/2)^2); % (slugs/ft^2) mass moment of inertia x when fuel is in the wingtips
c_lda_f = .039; % aileron effectiveness control derivative
c_lda_right_f = .0195; % aileron effectiveness control derivative

figure(2);
title('Max Roll Rate for the Navion and F-104A')
xlabel('Airspeed(ft/s)')
ylabel('Max Roll Rate (rad/s)')
grid
figure(3);
title('Open Loop Step Response vs Airspeed for the Navion and F-104A')
xlabel('Time(s)')
ylabel('Open Loop Step Response(rad/s)')
grid
figure(5);
title('Right Aileron Failure Open Loop Step Response vs Airspeed for the Navion and F-104A')
xlabel('Time(s)')
ylabel('Open Loop Step Response(rad/s)')
grid
figure(7);
title('Open Loop Step Response vs Airspeed for a fuel in wingtips Navion and F-104A')
xlabel('time(s)')
ylabel('Open Loop Step Response(rad/s)')
grid

% creating state spaces & step responses for F-104A & Navion %%problem 3 %
for v = [84:30:338]
    q = (rho * v.^2)/2; % air dynamic pressure
    L_p_n = ((q * s_n * b_n^2 * c_lp_n) / (2 * I_x_n * v )); % (1/s) roll damping 
    L_da_n = (q * s_n * b_n * c_lda_n) / I_x_n; % (1/s^2) control effectiveness
    L_da_right_n = (q * s_n * b_n * c_lda_right_n) / I_x_n; % (1/s^2) control effectiveness
    L_p_wingtips_n = ((s_n * b_n^2 * c_lp_n) / (2 * I_x_wingtips_n ))*((rho * v)/2); % (1/s) roll damping 
    L_da_wingtips_n = (q * s_n * b_n * c_lda_n) / I_x_wingtips_n; % (1/s^2) control effectiveness
    L_p_f = ((s_f * b_f^2 * c_lp_f) / (2 * I_x_f ))*((rho * v)/2);  % (1/s) roll damping
    L_da_f = (q * s_f * b_f * c_lda_f) / I_x_f; % (1/s^2) control effectiveness
    L_da_right_f = (q * s_f * b_f * c_lda_right_f) / I_x_f; % (1/s^2) control effectiveness
    L_p_wingtips_f = ((s_f * b_f^2 * c_lp_f) / (2 * I_x_wingtips_n ))*((rho * v)/2); % (1/s) roll damping 
    L_da_wingtips_f = (q * s_f * b_f * c_lda_right_f) / I_x_wingtips_n; % (1/s^2) control effectiveness
    
    sys_f(v) = ss(L_p_f, L_da_f, 1, 0);
    step_f = step(sys_f(v));
    sys_n(v) = ss(L_p_n, L_da_n, 1, 0);
    step_n = -step(sys_n(v));
    
    sys_right_n(v) = ss(L_p_n, L_da_right_n, 1, 0);
    step_right_n = -step(sys_right_n(v));
    sys_right_f(v) = ss(L_p_f, L_da_right_f, 1, 0);
    step_right_f = step(sys_right_f(v));
    
    sys_wingtips_f(v) = ss(L_p_wingtips_f, L_da_wingtips_f, 1, 0);
    step_wingtips_f = step(sys_wingtips_f(v));
    sys_wingtips_n(v) = ss(L_p_wingtips_n, L_da_wingtips_n, 1, 0);
    step_wingtips_n = -step(sys_wingtips_n(v));
    
    % max roll rate  %

	p_max_n = (L_da_n ./ L_p_n) .* da
    p_max_f = -(L_da_f ./ L_p_f) .* da
     
    % problem 2 %
    figure(2);
    hold on
    plot(v, p_max_n,'--b')
    plot(v, p_max_f,'--r')
    legend({'navion','F-104A'},'Location','south')
    hold off
    
    % Problem 3 %
    figure(3);
    hold on
    plot(step_n,'--b')
    plot(step_f,'--r')
    legend({'navion 84-338(ft/s)','F-104 84-338(ft/s)'},'Location','south')
    hold off
    
    % Problem 4 part 2%
    figure(5);
    hold on
    plot(step_right_n,'--b')
    plot(step_right_f,'--r')
    legend({'navion 84-338(ft/s)','F-104 84-338(ft/s)'},'Location','south')
    hold off
    
    % Problem 5 part 2%
    figure(7);
    hold on
    plot(step_wingtips_n,'--b')
    plot(step_wingtips_f,'--r')
    legend({'navion 84-338(ft/s)','F-104 84-338(ft/s)'},'Location','south')
    hold off
end