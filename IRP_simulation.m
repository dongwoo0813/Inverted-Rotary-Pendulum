% Simulation parameters
dt = 0.01; % Time step (s)
T = 10;    % Total simulation time (s)
n = T / dt; % Number of steps

% Initial conditions
theta = 0; % Arm angle (rad)
alpha = pi/4; % Pendulum angle (rad)
theta_dot = 0; % Arm angular velocity (rad/s)
alpha_dot = 0; % Pendulum angular velocity (rad/s)

% Control input (for testing)
u = 0;

% Data storage
results = zeros(n, 4); % [theta, alpha, theta_dot, alpha_dot]

% Time integration loop
for i = 1:n
    % Compute accelerations
    theta_ddot_val = theta_ddot_func(theta, alpha, theta_dot, alpha_dot, u);
    alpha_ddot_val = alpha_ddot_func(theta, alpha, theta_dot, alpha_dot, u);
    
    % Update states using Euler integration
    theta_dot = theta_dot + theta_ddot_val * dt;
    alpha_dot = alpha_dot + alpha_ddot_val * dt;
    theta = theta + theta_dot * dt;
    alpha = alpha + alpha_dot * dt;
    
    % Store results
    results(i, :) = [theta, alpha, theta_dot, alpha_dot];
end

% Plot results
time = linspace(0, T, n);
figure;
subplot(2, 1, 1);
plot(time, results(:, 1), 'r', time, results(:, 2), 'b');
legend('Arm Angle (theta)', 'Pendulum Angle (alpha)');
xlabel('Time (s)');
ylabel('Angle (rad)');
title('Angles');

subplot(2, 1, 2);
plot(time, results(:, 3), 'r', time, results(:, 4), 'b');
legend('Arm Angular Velocity (theta\_dot)', 'Pendulum Angular Velocity (alpha\_dot)');
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
title('Angular Velocities');