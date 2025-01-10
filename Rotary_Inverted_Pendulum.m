% Parameters
m_p = 0.5;      % Pendulum mass (kg)
m_a = 0.3;      % Arm mass (kg)
L_a = 0.4;      % Length of the arm (m)
L_p = 0.6;      % Length of the pendulum (m)
g = 9.81;       % Gravity (m/s^2)
I_a = (1/3) * m_a * L_a^2; % Arm moment of inertia
I_p = (1/3) * m_p * L_p^2; % Pendulum moment of inertia

% Equations of Motion
syms theta alpha theta_dot alpha_dot theta_ddot alpha_ddot u real

% Compute trigonometric terms for simplicity
sin_alpha = sin(alpha);
cos_alpha = cos(alpha);
sin_theta_alpha = sin(theta - alpha);
cos_theta_alpha = cos(theta - alpha);

% Equations
eq1 = (I_a + m_p * L_a^2) * theta_ddot + ...
      m_p * L_a * (L_p / 2) * alpha_ddot * cos_theta_alpha - ...
      m_p * L_a * (L_p / 2) * alpha_dot^2 * sin_theta_alpha == u;

eq2 = I_p * alpha_ddot + ...
      m_p * g * (L_p / 2) * sin_alpha - ...
      m_p * L_a * (L_p / 2) * theta_ddot * cos_theta_alpha == 0;

% Solve for theta_ddot and alpha_ddot
sol = solve([eq1, eq2], [theta_ddot, alpha_ddot]);

% Extract solutions
theta_ddot_sol = simplify(sol.theta_ddot);
alpha_ddot_sol = simplify(sol.alpha_ddot);

% Display results
disp('Theta double dot (Arm angular acceleration):');
disp(theta_ddot_sol);

disp('Alpha double dot (Pendulum angular acceleration):');
disp(alpha_ddot_sol);

% Create MATLAB functions for dynamics
matlabFunction(theta_ddot_sol, 'File', 'theta_ddot_func.m', 'Vars', [theta, alpha, theta_dot, alpha_dot, u]);
matlabFunction(alpha_ddot_sol, 'File', 'alpha_ddot_func.m', 'Vars', [theta, alpha, theta_dot, alpha_dot, u]);