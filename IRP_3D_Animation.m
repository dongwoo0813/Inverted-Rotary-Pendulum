% Simulation parameters
dt = 0.001; % Time step (s)
T = 5;    % Total simulation time (s)
n = floor(T / dt); % Number of steps

% Initial conditions
theta = 0; % Arm angle (rad)
alpha = (180/180)*pi; % Pendulum angle (rad)
theta_dot = 0; % Arm angular velocity (rad/s)
alpha_dot = 0.01; % Pendulum angular velocity (rad/s)

% Desired Alpha
alpha_des = pi;
alpha_dot_des = 0;


% Control input (for testing)
integral_error = 0;
u = 0;

% Control gains for PD controller
K_p = 1;  % Proportional gain
K_d = 0.1;   % Derivative gain
K_i = 0.001; % Integral Gain


% Parameters
L_a = 0.4; % Length of the arm (m)
L_p = 0.6; % Length of the pendulum (m)

% Data storage
results = zeros(n, 4); % [theta, alpha, theta_dot, alpha_dot]

% Time integration loop
for i = 1:n

% Calculate the error (target is alpha = pi, which is the vertical down position)
    error = alpha - alpha_des;  % Error = current angle - target angle (pi)
    
    % Calculate the integral of the error (summation over time)
    integral_error = integral_error + error * dt;
    
    % Calculate the derivative of the error (rate of change of the error)
    derivative_error = alpha_dot - alpha_dot_des;
    
    % Calculate the control input (torque) using PID control
    u = -K_p * error - K_i * integral_error - K_d * derivative_error;  % PID control law
    

    % Compute accelerations
    theta_ddot_value = theta_ddot_func(theta, alpha, theta_dot, alpha_dot, u);
    alpha_ddot_value = alpha_ddot_func(theta, alpha, theta_dot, alpha_dot, u);
    
    % Update states using Euler integration
    theta_dot = theta_dot + theta_ddot_value * dt;
    alpha_dot = alpha_dot + alpha_ddot_value * dt;
    theta = theta + theta_dot * dt;
    alpha = alpha + alpha_dot * dt;
    
    % Store results
    results(i, :) = [theta, alpha, theta_dot, alpha_dot];
end

% Animation
figure;
hold on;
grid on;
axis equal;
axis([-1.5, 1.5, -1.5, 1.5, -1, 1.5]); % Set axis limits
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('Inverted Rotary Pendulum 3D Animation');

% Set 3D view angle
view(45, 30); % Azimuth: 45 degrees, Elevation: 30 degrees

% Base rotation
base = line([0, 0], [0, 0], [0, 0], 'Color', 'k', 'LineWidth', 2); 

% Arm
arm_line = line([0, 0], [0, 0], [0, 0], 'Color', 'r', 'LineWidth', 2);

% Pendulum
pendulum_line = line([0, 0], [0, 0], [0, 0], 'Color', 'b', 'LineWidth', 2);

% Animation loop
for i = 1:n
    % Extract angles
    theta = results(i, 1); % Arm angle (rotation around Z-axis)
    alpha = results(i, 2); % Pendulum angle (rotation around arm)

    % Calculate positions
    arm_x = L_a * cos(theta);
    arm_y = L_a * sin(theta);
    arm_z = 0;

    pendulum_x = arm_x + L_p * sin(alpha) * cos(theta);
    pendulum_y = arm_y + L_p * sin(alpha) * sin(theta);
    pendulum_z = -L_p * cos(alpha);

    % Update base rotation (if needed)
    set(base, 'XData', [0, 0], 'YData', [0, 0], 'ZData', [0, 0]);

    % Update arm line
    set(arm_line, 'XData', [0, arm_x], 'YData', [0, arm_y], 'ZData', [0, arm_z]);

    % Update pendulum line
    set(pendulum_line, 'XData', [arm_x, pendulum_x], ...
                       'YData', [arm_y, pendulum_y], ...
                       'ZData', [arm_z, pendulum_z]);

    % Pause for animation effect
    pause(dt);
end
