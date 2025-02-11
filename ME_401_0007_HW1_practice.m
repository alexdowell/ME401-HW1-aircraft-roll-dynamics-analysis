% Alexander Dowell
% 2/1/22 HW1 
% Write a Matlab script, function, or diagram to define and simulate roll convergence dynamics.

% p_dot = ( L_p * p ) + ( L_del_a * del_a )

% Use the Navion and F-104 Starfighter stability and control derivatives, mass properties, and dimensions.  You can also add provisions for including other aircraft dynamic models into your code. 

% Airspeed range: 50 - 200 knots

% Generate the results:

% 1 - How do the derivatives L_p and L_delta_a change over the airspeed range for each aircraft?

% 2 - How does the maximum roll rate change for each aircraft?  Assume delta_a_max is 25 degrees

% 3 - How does the open-loop step response change over the airspeed range?

% 4 - What happens during a partial aileron failure?  (Fail the left or right aileron and compare maximum roll rate and open-loop step response)

% 5 - What happens if the wingtip tanks are full? (Compare maximum roll rate and open-loop step response)

v = 84 : 15 : 338; % (ft/s) air speed
rho = .0023769; % (slugs/m^3) air density at sea level and 60 deg F
q = (rho * v.^2)/2; % air dynamic pressure
da = deg2rad(25);

% navion %
s_n = 184; % (ft^2) wing surface area 
b_n = 33.4; % (ft) wing span 
c_lp_n = -.410; % roll damping stability derivative 
I_x_n = 1048; % (slugs/ft^2) mass moment of inertia x
I_x_wingtips_n = 1048; % (slugs/ft^2) mass moment of inertia x when fuel is in the wingtips
c_lda_n = -.134; % aileron effectiveness control derivative
c_lda_right_n = -.067; % right aileron effectiveness control derivative
w_n = 2750; % (lbs) weight
L_p_n = ((s_n * b_n^2 * c_lp_n) / (2 * I_x_n ))*((rho * v)/2); % (1/s) roll damping 
L_da_n = (q * s_n * b_n * c_lda_n) / I_x_n; % (1/s^2) control effectiveness
L_da_right_n = (q * s_n * b_n * c_lda_right_n) / I_x_n; % (1/s^2) control effectiveness
L_p_wingtips_n = ((s_n * b_n^2 * c_lp_n) / (2 * I_x_wingtips_n ))*((rho * v)/2); % (1/s) roll damping 
L_da_wingtips_n = (q * s_n * b_n * c_lda_n) / I_x_wingtips_n; % (1/s^2) control effectiveness

% creating A,B,C, and D matrices for navion %
A_n = eye(length(v));
B_n = eye(length(v));
C_n = eye(length(v));

for i = [1:1:length(v)];
   A_n(i,i) = L_p_n(i);
   B_n(i,i) = L_da_n(i);
end

% creating state space for navion %
sys_n = ss(A_n, B_n, C_n, 0);

% step response form state space navion%
step_n_p = step(sys_n);

% creating transfer functions navion %
for i = [1:1:length(v)];
    systf_n(i) = tf(L_da_n(i),[L_p_n(i) 1]);
end

% max roll rate navion %
for i = [1:1:length(v)];
	p_max_n(i) = -(L_da_n(i) / L_p_n(i)) * da;
end

% step response navion %
for i = [1:1:length(v)];
	step_n = step(-systf_n(i));
end

% creating right aileron failure A,B,C, and D matrices for navion %
A_right_n = eye(length(v));
B_right_n = eye(length(v));
C_right_n = eye(length(v));

for i = [1:1:length(v)];
   A_right_n(i,i) = L_p_n(i);
   B_right_n(i,i) = L_da_right_n(i);
end

% creating right aileron failure state space for navion %
sys_right_n = ss(A_right_n, B_right_n, C_right_n, 0);

% right aileron failure transfer functions navion %
for i = [1:1:length(v)];
    systf_right_n(i) = tf(L_da_right_n(i),[L_p_n(i) 1]);
end

% right aileron failure max roll failure navion %
for i = [1:1:length(v)];
        p_max_right_n(i) = -(L_da_right_n(i) / L_p_n(i)) * da;
end

% right aileron failure step response navion %
for i = [1:1:length(v)];
	step_right_n = step(-systf_right_n(i));
end
% creating A,B,C, and D matrices for fuel in wingtips navion %
A_wingtips_n = eye(length(v));
B_wingtips_n = eye(length(v));
C_wingtips_n = eye(length(v));

for i = [1:1:length(v)];
   A_wingtips_n(i,i) = L_p_wingtips_n(i);
   B_wingtips_n(i,i) = L_da_wingtips_n(i);
end

% creating state space for fuel in wingtips navion %
sys_wingtips_n = ss(A_wingtips_n, B_wingtips_n, C_wingtips_n, 0);

% creating transfer functions for fuel in wingtips navion %
for i = [1:1:length(v)];
    systf_wingtips_n(i) = tf(L_da_wingtips_n(i),[L_p_wingtips_n(i) 1]);
end

% max roll rate for fuel in wingtips navion %
for i = [1:1:length(v)];
	p_max_wingtips_n(i) = -(L_da_wingtips_n(i) / L_p_wingtips_n(i)) * da;
end

% step response for fuel in wingtips navion %
for i = [1:1:length(v)];
	step_wingtips_n = step(-systf_wingtips_n(i));
end

% F-104A %
s_f = 196.1; % (ft^2) wing surface area 
b_f = 21.94; % (ft) wing span 
c_lp_f = -.285; % roll damping stability derivative 
I_x_f = 3549; % (slugs/ft^2) mass moment of inertia x
I_x_wingtips_f = 3549; % (slugs/ft^2) mass moment of inertia x when fuel is in the wingtips
c_lda_f = .039; % aileron effectiveness control derivative
c_lda_right_f = .0195; % aileron effectiveness control derivative 
w_f = 16300; % (lbs) weight
L_p_f = ((s_f * b_f^2 * c_lp_f) / (2 * I_x_f ))*((rho * v)/2);  % (1/s) roll damping
L_da_f = (q * s_f * b_f * c_lda_f) / I_x_f; % (1/s^2) control effectiveness
L_da_right_f = (q * s_f * b_f * c_lda_right_f) / I_x_f; % (1/s^2) control effectiveness
L_p_wingtips_f = ((s_f * b_f^2 * c_lp_f) / (2 * I_x_wingtips_n ))*((rho * v)/2); % (1/s) roll damping 
L_da_wingtips_f = (q * s_f * b_f * c_lda_right_f) / I_x_wingtips_n; % (1/s^2) control effectiveness

% creating A,B,C, and D matrices for F-104A %
A_f = eye(length(v));
B_f = eye(length(v));
C_f = eye(length(v));

for i = [1:1:length(v)];
   A_f(i,i) = L_p_f(i);
   B_f(i,i) = L_da_f(i);
end

% creating state space for F-104A %
sys_f = ss(A_f, B_f, C_f, 0);

% step response form state space F-104A%
step_f_p = step(sys_f);

% creating transfer functions F-104A %
for i = [1:1:length(v)];
    systf_f(i) = tf(L_da_f(i),[L_p_f(i) 1]);
end

% max roll rate F-104A %
for i = [1:1:length(v)];
	p_max_f(i) = -(L_da_f(i) / L_p_f(i)) * da;
end

% step response F-104A %
for i = [1:1:length(v)];
	step_f = step(-systf_f(i));
end

% creating right aileron failure A,B,C, and D matrices for F-104A %
A_right_f = eye(length(v));
B_right_f = eye(length(v));
C_right_f = eye(length(v));

for i = [1:1:length(v)];
   A_right_f(i,i) = L_p_f(i);
   B_right_f(i,i) = L_da_right_f(i);
end

% creating right aileron failure state space for F-104A %
sys_right_f = ss(A_right_f, B_right_f, C_right_f, 0);

% right aileron failure transfer functions F-104A %
for i = [1:1:length(v)];
    systf_right_f(i) = tf(L_da_right_f(i),[L_p_f(i) 1]);
end

% max roll rate with right aileron failure F-104A %
for i = [1:1:length(v)];
        p_max_right_f(i) = -(L_da_right_f(i) / L_p_f(i)) * da;
end

% right aileron failure step response F-104A %
for i = [1:1:length(v)];
	step_right_f = step(-systf_right_f(i));
end

% creating A,B,C, and D matrices for fuel in wingtips F-104A %
A_wingtips_f = eye(length(v));
B_wingtips_f = eye(length(v));
C_wingtips_f = eye(length(v));

for i = [1:1:length(v)];
   A_wingtips_f(i,i) = L_p_wingtips_f(i);
   B_wingtips_f(i,i) = L_da_wingtips_f(i);
end

% creating state space for fuel in wingtips F-104A %
sys_wingtips_f = ss(A_wingtips_f, B_wingtips_f, C_wingtips_f, 0);

% creating transfer functions for fuel in wingtips F-104A %
for i = [1:1:length(v)];
    systf_wingtips_f(i) = tf(L_da_wingtips_f(i),[L_p_wingtips_f(i) 1]);
end

% max roll rate for fuel in wingtips F-104A %
for i = [1:1:length(v)];
	p_max_wingtips_f(i) = -(L_da_wingtips_f(i) / L_p_wingtips_f(i)) * da;
end

% step response for fuel in wingtips F-104A %
for i = [1:1:length(v)];
	step_wingtips_f = step(-systf_wingtips_f(i));
end

% problem 1 %
figure(1);
title('L_p and L_d_a vs. Airspeed for the Navion and F-104A')
xlabel('Airspeed(ft/s)')
ylabel('L_p (1/s) and L_d_a (1/s^2)')
grid
hold on
plot(v,L_p_n,'--p')
plot(v,L_da_n,'--p')
plot(v,L_p_f,'--r')
plot(v,L_da_f,'--b')
legend({'L_p navion','L_d_a navion','L_p F-104A','L_d_a F-104A'},'Location','south')
hold off

% Problem 2 %
figure(2);
title('Max Roll Rate for the Navion and F-104A')
xlabel('Airspeed(ft/s)')
ylabel('Max Roll Rate (rad/s)')
grid
hold on
plot(v,p_max_n,'--p')
plot(v,p_max_f,'--r')
legend({'navion','F-104A'},'Location','south')
hold off

% Problem 3 %
figure(3);
title('Open Loop Step Response vs Airspeed for the Navion and F-104A')
xlabel('Airspeed(ft/s)')
ylabel('Open Loop Step Response(rad/s)')
grid
hold on
plot(step_n,'--b')
plot(step_f,'--p')
legend({'navion','F-104'},'Location','south')
hold off

% Problem 4 %
figure(4);
title('Right Aileron Failure Max Roll Rate for the Navion and F-104A')
xlabel('Airspeed(ft/s)')
ylabel('Max Roll Rate (rad/s)')
grid
hold on
plot(v,p_max_right_n,'--p')
plot(v,p_max_right_f,'--r')
legend({'navion','F-104A'},'Location','south')
hold off

figure(5);
title('Right Aileron Failure Open Loop Step Response vs Airspeed for the Navion and F-104A')
xlabel('Airspeed(ft/s)')
ylabel('Open Loop Step Response(rad/s)')
grid
hold on
plot(step_right_n,'--b')
plot(step_right_f,'--p')
legend({'navion','F-104'},'Location','south')
hold off

% Problem 5 %
figure(6);
title('Max Roll Rate for a fuel in wingtips Navion and F-104A')
xlabel('Airspeed(ft/s)')
ylabel('Max Roll Rate (rad/s)')
grid
hold on
plot(v,p_max_wingtips_n,'--p')
plot(v,p_max_wingtips_f,'--r')
legend({'navion','F-104A'},'Location','south')
hold off

figure(7);
title('Open Loop Step Response vs Airspeed for a fuel in wingtips Navion and F-104A')
xlabel('Airspeed(ft/s)')
ylabel('Open Loop Step Response(rad/s)')
grid
hold on
plot(step_wingtips_n,'--b')
plot(step_wingtips_f,'--p')
legend({'navion','F-104'},'Location','south')
hold off

% Practice %
figure(8);
title('Open Loop Step Response vs Airspeed for the Navion and F-104A')
xlabel('Airspeed(ft/s)')
ylabel('Open Loop Step Response(rad/s)')
grid
hold on
for i = [1:1:length(v)];
    plot(step_n_p(i),'--b')
    plot(step_f_p(i),'--p')
end
legend({'navion','F-104'},'Location','south')
hold off