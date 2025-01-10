% Simulation parameters
dt = 0.01; % Time step (s)
T = 10;    % Total simulation time (s)
n = T / dt; % Number of steps

% Initial conditions
theta = 0; % Arm angle (rad)
alpha = pi / 4; % Pendulum angle (rad)
theta_dot = 0; % Arm angular velocity (rad/s)
alpha_dot = 0; % Pendulum angular velocity (rad/s)

% Control input (for testing)
u = 0;

% Parameters
L_a = 0.4; % Length of the arm (m)
L_p = 0.6; % Length of the pendulum (m)

% Data storage
results = zeros(n, 4); % [theta, alpha, theta_dot, alpha_dot]

% Time integration loop
for i = 1:n
    % Compute accelerations
    theta_ddot = theta_ddot_func(theta, alpha, theta_dot, alpha_dot, u);
    alpha_ddot = alpha_ddot_func(theta, alpha, theta_dot, alpha_dot, u);
    
    % Update states using Euler integration
    theta_dot = theta_dot + theta_ddot * dt;
    alpha_dot = alpha_dot + alpha_ddot * dt;
    theta = theta + theta_dot * dt;
    alpha = alpha + alpha_dot * dt;
    
    % Store results
    results(i, :) = [theta, alpha, theta_dot, alpha_dot];
end

% Animation
figure;
hold on;
axis equal;
axis([-1.5, 1.5, -1, 1.5]); % Set axis limits
xlabel('X (m)');
ylabel('Y (m)');
title('Inverted Rotary Pendulum Animation');

% Arm and pendulum lines
arm_line = line([0, 0], [0, 0], 'Color', 'r', 'LineWidth', 2);
pendulum_line = line([0, 0], [0, 0], 'Color', 'b', 'LineWidth', 2);

% Animation loop
for i = 1:n
    % Extract angles
    theta = results(i, 1);
    alpha = results(i, 2);
    
    % Calculate positions
    arm_x = L_a * cos(theta);
    arm_y = L_a * sin(theta);
    pendulum_x = arm_x + L_p * sin(alpha);
    pendulum_y = arm_y - L_p * cos(alpha);
    
    % Update arm line
    set(arm_line, 'XData', [0, arm_x], 'YData', [0, arm_y]);
    
    % Update pendulum line
    set(pendulum_line, 'XData', [arm_x, pendulum_x], 'YData', [arm_y, pendulum_y]);
    
    % Pause for animation effect
    pause(dt);
end
