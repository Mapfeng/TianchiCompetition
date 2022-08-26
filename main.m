
x_now = 0;
y_now = 0;
v_now = 2;
a_now = 0;
theta_now = pi/4;
% omega_now = 0;
delta_f_now = 0;
L = 2.6;
T = 0.05;

% X_next = GetNextStateByCTRA(x_now, y_now, v_now, a_now, theta_now, omega_now, T);
[X_next] = GetNextStateByBicycleModel(x_now, y_now, theta_now, v_now, delta_f_now, a_now, T, L);
disp(X_next);
