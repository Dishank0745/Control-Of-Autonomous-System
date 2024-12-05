%% Notations
% xi = [x, y, z]
% nu_I = [x_dot, y_dot, z_dot] %inertial frame
% nu_B = [u, v, w] %body frame

% eta = [phi, theta, psi]
% omega_i = [phi_dot, theta_dot, psi_dot] % inertia frame
% omega_b = [p, q, r] % body frame
%% Prepare workspace
clear; close all; clc;
% Select Control Type
implement_control = 1; % Select 0 to simulate without control and 1 to simulate with control
% recall x and y are not observable so further modifications have to be made
% 1 - stabilization, 2 - stabilization with modifications for x and y
% 3 - trajectory tracking
% 4 - Feedback Linearization.
param.control_type = 3; 

% select trajectory type 
% 1 - Circular 2- Spiral
param.traj = 2; 

% Time change
% 1 - 1 * T, 2 - 2 * T, 3 - 3 * T
param.time_change = 3;
%% Quadcopter Variables
param.g = 9.81; 
param.m = 0.468;
param.l = 0.225;
param.Jr = 3.357e-5;
param.Kt = 2.98e-6;
param.Kb = 1.140e-7;
Kdx = 0.25; Kdy = 0.25; Kdz = 0.25;
param.Kd = diag([Kdx; Kdy; Kdz]);
Ixx = 4.856e-3; Iyy = 4.856e-3; Izz = 8.801e-3;
param.I = diag([Ixx; Iyy; Izz]);

%% Control Variable
% Nonlinear PD Gains
% Kd_phi = Kd_theta = Kd_psi = Kd
% Kp_phi = Kp_theta = Kp_psi = Kp
if param.control_type == 1
    if implement_control == 0
        PD_var.Kd = 0; PD_var.Kp = 0;
        PD_var.Kd_z = 0; PD_var.Kp_z = 0;
    else
        PD_var.Kd = 2.6; PD_var.Kp = 1.5;
        PD_var.Kd_z = 1.5; PD_var.Kp_z = 6;
        PD_var.Kp_xy = 0.04;
    end
else
    PD_var.Kd = 2.6; PD_var.Kp = 1.5;
    PD_var.Kd_z = 1.5; PD_var.Kp_z = 6;
    PD_var.Kp_xy = 0.04;
end
% For Nonlinear PD with helix trajectory
if param.control_type == 3 && param.traj == 2
    PD_var.Kd = 5; PD_var.Kp = 15;
    PD_var.Kd_z = 1.5; PD_var.Kp_z = 1;
    PD_var.Kp_xy = 0.05;
end
% Feedback controller gains
PD_var.FB = [-5.5 * ones(3,1);  -2 * ones(3,1)
             -5.2 * ones(3,1); -10 * ones(3,1)];
%% Initialize Simulation
start_time = 0;
T = 15; % in seconds
if param.control_type == 1
    end_time = T;
else
    end_time = param.time_change * T;
end
dt = 0.001;
sim_time = (start_time:dt:end_time)';
n_sim = length(sim_time);

xi =  zeros(n_sim, 3);
xi_desired =  zeros(n_sim, 3);
nu_I = zeros(n_sim, 3);
eta = zeros(n_sim, 3);
omega_I = zeros(n_sim, 3);
control_values = zeros(n_sim, 4);

% intitial states
if param.control_type == 1 || param.control_type == 2
    % since it is a stabilization control, we give it an initial condition
    xi(1, :) = [-1, 2, 1]; % position [m]
    eta(1, :) = deg2rad([10, -10, 5]); % angle [deg]
end
%% Simulation
for idx_t = 1:n_sim
    % extract current time step data
    xi_current = xi(idx_t, :)';
    nu_I_current = nu_I(idx_t, :)';
    eta_current = eta(idx_t, :)';
    omega_I_current = omega_I(idx_t, :)';
    
    % Obtain control input from controller
    if param.control_type == 1 || param.control_type == 2
        % desired position = 0
        xi_des = zeros(1, 3);
        xi_des_d = zeros(1, 3);
        xi_des_dd = zeros(1, 3);
    elseif param.control_type == 3
        % desired position is a circular or spiral trajectory
        if param.traj == 1 
            [xi_des, xi_des_d, xi_des_dd] = circular_traj(sim_time(idx_t), T);
        elseif param.traj == 2
            [xi_des, xi_des_d, xi_des_dd] = spiral_traj(sim_time(idx_t), T);
        end
    end
    
    % convert from inertia frame to body frame
    omega_B_current = inertia2body(omega_I_current, eta_current);
    
    % Select control algorithm
     control_input = PD(omega_I_current, eta_current, xi_current, nu_I_current, param, PD_var, xi_des, xi_des_d, xi_des_dd); % omega square
    
    
    nu_I_dot = linear_acceleration(control_input, eta_current, nu_I_current, param);
    omega_B_dot = rotational_acceleration(control_input, omega_B_current, param);
    
    % linear motion evolution using discretized Euler
    nu_I_next = nu_I_current + dt * nu_I_dot;
    xi_next = xi_current + dt * nu_I_next;
    
    % rotational motion evolution using discretized Euler
    omega_B_next = omega_B_current + dt * omega_B_dot;
    % convert from bodyframe to inertia frame
    omega_I_next = body2inertia(omega_B_next, eta_current);
    eta_next = eta_current + dt * omega_I_next;
    
    % fill in next time step data
    xi(idx_t + 1, :) = xi_next';
    nu_I(idx_t + 1, :) = nu_I_next';
    eta(idx_t + 1, :) = eta_next';
    omega_I(idx_t + 1, :) = omega_I_next';
    control_values(idx_t, :) = sqrt(control_input)';
    
    if param.control_type == 3
        xi_desired(idx_t, :) = xi_des';
    end
end

%% Plots
figure
hold on
plot(sim_time, control_values(:, 1), 'LineWidth', 2)
plot(sim_time, control_values(:, 2), 'LineWidth', 2)
plot(sim_time, control_values(:, 3), 'LineWidth', 2)
plot(sim_time, control_values(:, 4), 'LineWidth', 2)
hold off
xlabel('Time [s]')
ylabel('control input \omega[rad/s]')
legend('\omega_1', '\omega_2', '\omega_3', '\omega_4', 'Location', 'Best')
grid on

if param.control_type == 1 || param.control_type == 2
    figure
    hold on
    plot(sim_time, xi(1:end-1, 1), 'LineWidth', 2)
    plot(sim_time, xi(1:end-1, 2), 'LineWidth', 2)
    plot(sim_time, xi(1:end-1, 3), 'LineWidth', 2)
    hold off
    xlabel('Time [s]')
    ylabel('positions [m]')
    legend('x', 'y', 'z', 'Location', 'Best')
    grid on
    
    figure
    hold on
    plot(sim_time, rad2deg(eta(1:end-1, 1)), 'LineWidth', 2)
    plot(sim_time, rad2deg(eta(1:end-1, 2)), 'LineWidth', 2)
    plot(sim_time, rad2deg(eta(1:end-1, 3)), 'LineWidth', 2)
    hold off
    xlabel('Time [s]')
    ylabel('angles [deg]')
    legend('\phi', '\theta', '\psi', 'Location', 'Best')
    grid on
    
elseif param.control_type == 3
    figure
    hold on
    plot(sim_time, xi(1:end-1, 1), 'LineWidth', 2)
    plot(sim_time, xi_desired(:, 1), 'LineWidth', 2)
    hold off
    xlabel('Time [s]')
    ylabel('position - x [m]')
    legend('x actual', 'x desired', 'Location', 'Best')
    grid on
    
    figure
    hold on
    plot(sim_time, xi(1:end-1, 2), 'LineWidth', 2)
    plot(sim_time, xi_desired(:, 2), 'LineWidth', 2)
    hold off
    xlabel('Time [s]')
    ylabel('position - y [m]')
    legend('y actual', 'y desired', 'Location', 'Best')
    grid on
    
    
    figure
    hold on
    plot3(xi(1:end-1, 1), xi(1:end-1, 2), xi(1:end-1, 3), 'LineWidth', 2)
    plot3(xi_desired(1:end-1, 1), xi_desired(1:end-1, 2), xi_desired(1:end-1, 3),'--', 'LineWidth', 2)
    plot3(xi_desired(1,1), xi_desired(1,2), xi_desired(1,3),'o', 'LineWidth', 6,'MarkerSize', 6)
    plot3(xi(1,1), xi(1,2), xi(1,3),'*', 'LineWidth', 6,'MarkerSize', 6)
    
    xlabel('x [m]')
    ylabel('y [m]')
    zlabel('z [m]')
    grid on
    grid minor
    legend('Actual Trajectory','Desired Trajectory','Start of desired trajectory','Start of actual trajectory')
end

%% Functions
function nu_I_dot = linear_acceleration(control_input, eta, nu_I, param)
    % Compute translational/linear motion using Newtonian method
    % gravity
    nu_I_dot_1 = -param.g * [0, 0, 1]';
    R = rotation(eta);
    % total thrust
    T_B = [0, 0, param.Kt * sum(control_input)]';
    nu_I_dot_2 =  (1 / param.m) * R * T_B;
    % drag force
    nu_I_dot_3 = -(1 / param.m) * param.Kd * nu_I;
    
    nu_I_dot = nu_I_dot_1 + nu_I_dot_2 + nu_I_dot_3;
end

function omega_B_dot = rotational_acceleration(control_input, omega_B, param)
    % Compute rotational motion using Euler method
    tau = torques(control_input, param);
    gamma = computeGamma(control_input, omega_B, param);
    % using direct representation for ease of computation
    omega_B_dot = inv(param.I) * (tau - gamma + cross(omega_B, param.I * omega_B));
end

function gamma = computeGamma(control_input, omega_B, param)
    % compute the gyroscopic forces causes by combined rotation of motors
    omega = sqrt(control_input);
    omega_gamma = -omega(1) + omega(2) - omega(3) + omega(4);
    gamma = param.Jr * cross(omega_B, [0; 0; 1]) * omega_gamma;
end

% Compute torques, given current inputs, length, drag coefficient, and thrust coefficient.
function tau = torques(control_input, param)
    % Inputs are values for ${\omega_i}^2$
    tau = [
        param.l * param.Kt * (-control_input(2) + control_input(4))
        param.l * param.Kt * (-control_input(1) + control_input(3))
        param.Kb * (-control_input(1) + control_input(2) - control_input(3) + control_input(4))
    ];
end
function R = rotation(eta)
    % Compute rotational matrix
    phi = eta(1); theta = eta(2); psi = eta(3);
    
    R11 = cos(psi) * cos(theta);
    R12 = cos(psi) * sin(theta) * sin(phi) - sin(psi) * cos(phi);
    R13 = cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(phi);
    
    R21 = sin(psi) * cos(theta);
    R22 = sin(psi) * sin(theta) * sin(phi) + cos(psi) * cos(phi);
    R23 = sin(psi) * sin(theta) * cos(phi) - cos(psi) * sin(phi);
    
    R31 = -sin(theta);
    R32 = cos(theta) * sin(phi);
    R33 = cos(theta) * cos(phi);
    
    R = [R11 R12 R13;...
         R21 R22 R23;...
         R31 R32 R33];
end

function I2B = inertia2body(omega_I, eta)
    phi = eta(1); theta = eta(2); psi = eta(3);
    W = [1   0        -sin(theta);...
         0   cos(phi)  cos(theta) * sin(phi);...
         0  -sin(phi)  cos(theta) * cos(phi)];
    I2B = W * omega_I;
end

function B2I = body2inertia(omega_B, eta)
    phi = eta(1); theta = eta(2); psi = eta(3);
    W_inv = [1   sin(phi) * tan(theta)    cos(phi) * tan(theta);...
             0   cos(phi)                -sin(phi);...
             0   sin(phi) / cos(theta)    cos(phi) / cos(theta)];
    B2I = W_inv * omega_B;  
end

%% Controls
function ctrl = PD(omega_I, eta, xi, nu_I, param, PD_var, xi_des, xi_des_d, xi_des_dd)
    ctrl = zeros(4, 1);
    % Unload parameters
    phi = eta(1); theta = eta(2); psi = eta(3);
    phi_d = omega_I(1); theta_d = omega_I(2); psi_d = omega_I(3);
    z = xi(3); z_d = nu_I(3);
    
    % desired position computation - x and y desired translated to phi and theta
    z_des = xi_des(3); z_des_d = xi_des_d(3);
    % desired angle
    if param.control_type == 2 || param.control_type == 3
        eta_des = pos2ang(xi, xi_des, xi_des_d, xi_des_dd, nu_I, param, PD_var);
    else
        eta_des = zeros(size(eta));
    end
    % desired angle derivative
    eta_des_d = zeros(size(eta));
    % unload values
    phi_des = eta_des(1); phi_des_d = eta_des_d(1); 
    theta_des = eta_des(2); theta_des_d = eta_des_d(2);
    psi_des = eta_des(3); psi_des_d = eta_des_d(3);

    T = (param.g + PD_var.Kd_z * (z_des_d - z_d) + PD_var.Kp_z * (z_des - z)) * ( param.m / (cos(phi) * cos(theta)) );
    tau_phi = (PD_var.Kd * (phi_des_d - phi_d) + PD_var.Kp * (phi_des - phi)) * param.I(1,1);
    tau_theta = (PD_var.Kd * (theta_des_d - theta_d) + PD_var.Kp * (theta_des - theta)) * param.I(2,2);
    tau_psi = (PD_var.Kd * (psi_des_d - psi_d) + PD_var.Kp * (psi_des - psi)) * param.I(3,3);
    
    ctrl(1) = ( T / (4 * param.Kt) ) - ( tau_theta / (2 * param.Kt * param.l) ) - ( tau_psi / (4 * param.Kb) );
    ctrl(2) = ( T / (4 * param.Kt) ) - ( tau_phi / (2 * param.Kt * param.l) ) + ( tau_psi / (4 * param.Kb) );
    ctrl(3) = ( T / (4 * param.Kt) ) + ( tau_theta / (2 * param.Kt * param.l) ) - ( tau_psi / (4 * param.Kb) );
    ctrl(4) = ( T / (4 * param.Kt) ) + ( tau_phi / (2 * param.Kt * param.l) ) + ( tau_psi / (4 * param.Kb) );
end

%% Desired Trajectory
% position trajectory
function [xi_des, xi_des_d, xi_des_dd]  = circular_traj(t, T)
    radius = 3;
    ang = 2 * pi * t/T;
    
    xi_des = zeros(3, 1);
    xi_des_d = zeros(3, 1);
    xi_des_dd = zeros(3, 1);
    % define circular trajectory for the x and y direction
    xi_des(1) = radius * cos(ang);
    xi_des(2) = radius * sin(ang);
    xi_des(3) = t/T; %  equation of line
    % define derivative
    xi_des_d(1) = -2 * pi * radius * sin(ang)/T;
    xi_des_d(2) = 2 * pi * radius * cos(ang)/T;
    xi_des_d(3) = 1/T;
    % define double derivative
    xi_des_dd(1) = -4 * radius * pi * pi * cos(ang) / (T * T);
    xi_des_dd(2) = -4 * radius * pi * pi * sin(ang) / (T * T);
    xi_des_dd(3) = 0;
end

function [xi_des, xi_des_d, xi_des_dd]  = spiral_traj(t, T)
    radius = 3;
    ang = 2* pi * t/T;
    h = 5;
    
    xi_des = zeros(3, 1);
    xi_des_d = zeros(3, 1);
    xi_des_dd = zeros(3, 1);
    % define circular trajectory for the x and y direction
    xi_des(1) = radius * cos(ang);
    xi_des(2) = h * t/T;
    xi_des(3) = t/T; %  equation of line
    % define derivative
    xi_des_d(1) = -2 * pi * radius * sin(ang)/T;
    xi_des_d(2) = h / T;
    xi_des_d(3) = 1/T;
    % define double derivative
    xi_des_dd(1) = -4 * radius * pi * pi * cos(ang) / (T * T);
    xi_des_dd(2) = 0;
    xi_des_dd(3) = 0;
end

% convert position trajectory to angle
function eta_des = pos2ang(xi, xi_des, xi_des_d, xi_des_dd, nu_I, param, PD_var)
    x_des = xi_des(1); x_des_d = xi_des_d(1); x_des_dd = xi_des_dd(1);
    y_des = xi_des(2); y_des_d = xi_des_d(2); y_des_dd = xi_des_dd(2);
    z_des = xi_des(3); z_des_d = xi_des_d(3); z_des_dd = xi_des_dd(3);
    
    x = xi(1); x_d = nu_I(1);
    y = xi(2); y_d = nu_I(2);
    z = xi(3); z_d = nu_I(3);
    
    % Psi_des will be fixed to zero
    eta_des = zeros(3, 1);
    
    T_des = param.m * ( z_des_dd + param.g) + param.Kd(3, 3) * z_des_d; 
    eta_des(1) = -(param.m * y_des_dd + param.Kd(2, 2) * y_des_d + PD_var.Kp_xy * (y_des - y)) / T_des;
    eta_des(2) = (param.m * x_des_dd + param.Kd(1, 1) *  x_des_d  + PD_var.Kp_xy * (x_des - x)) / T_des;
end
