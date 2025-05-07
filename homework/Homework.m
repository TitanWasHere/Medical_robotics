clear; clc; close all;

syms q1 q2 l1 l2 real
syms dq1 dq2 ddq1 ddq2 real
syms lambda dlambda real
syms theta_d_sym real % Symbolic placeholder for theta_d
syms p_trocar_x_sym p_trocar_y_sym real % Symbolic placeholders for p_trocar

%% Definition of values from user
lambda0 = 0.5;
theta_d_val = pi/8; % Numerical value for theta_d
q0 = [pi/4, 0];
l_val = [1,1]; % Numerical values for l1, l2

%% Symbolic values
q_sym = [q1;q2]; % Ensure it's a column vector for jacobian
dq_sym = [dq1;dq2];

p1_sym = [l1 * cos(q1) ; l1 * sin(q1)];
p2_sym = [l1 * cos(q1) + l2 * cos(q1+q2) ; l1 * sin(q1) + l2 * sin(q1+q2)];

theta_sym = q1 + q2;
J_theta_sym = jacobian(theta_sym, q_sym); % Will be [1, 1]

p_rcm_sym = p1_sym + lambda * (p2_sym - p1_sym);
J_rcm_q_sym = jacobian(p_rcm_sym, q_sym); % Jacobian of p_rcm w.r.t. q

dJ_rcm_lambda_sym = p2_sym - p1_sym; % This is d(p_rcm)/d(lambda)
J_rcm_ext_sym = [J_rcm_q_sym , dJ_rcm_lambda_sym]; % Jacobian of p_rcm w.r.t. [q; lambda]

%% Initial values (Numerical)
p_rcm_sym_with_lengths = subs(p_rcm_sym, {l1, l2}, {l_val(1), l_val(2)});
p_trocar_val = double(subs(p_rcm_sym_with_lengths, {q1,q2,lambda}, {q0(1), q0(2), lambda0}));

%% Kinematic control (Symbolic)
J_ext_sym = [J_theta_sym, sym(0) ; J_rcm_ext_sym];

p_trocar_target_sym = [p_trocar_x_sym; p_trocar_y_sym];
e_theta_sym = theta_d_sym - theta_sym;
e_rcm_sym = p_trocar_target_sym - p_rcm_sym;
e_ext_sym = [e_theta_sym ; e_rcm_sym];

% Gain matrices
k_rcm = 5;
k_theta = 3;
K_ext = diag([k_theta, k_rcm, k_rcm]);

% Null-space projector term
w_sym = [0; 0; 0.5 * (0.5 - lambda)];

% Task velocity (desired)
dtheta_d_val = 0;
dp_rcm_d_val = [0;0];
dx_d_val = [dtheta_d_val ; dp_rcm_d_val];

%% Convert symbolic expressions to MATLAB functions for speed
p1_fun = matlabFunction(p1_sym, 'Vars', {q1, l1});
p2_fun = matlabFunction(p2_sym, 'Vars', {q1, q2, l1, l2});
p_rcm_fun = matlabFunction(p_rcm_sym, 'Vars', {q1, q2, l1, l2, lambda});
theta_fun = matlabFunction(theta_sym, 'Vars', {q1, q2});
J_ext_fun = matlabFunction(J_ext_sym, 'Vars', {q1, q2, l1, l2, lambda});
e_ext_fun = matlabFunction(e_ext_sym, 'Vars', {q1, q2, l1, l2, lambda, theta_d_sym, p_trocar_target_sym});
w_fun = matlabFunction(w_sym, 'Vars', {lambda});

%% Simulation Parameters
dt = 0.02;       % Time step
T_sim = 10;      % Total simulation time
N_steps = round(T_sim/dt);

% Initial conditions for simulation state
q_curr = q0(:);
lambda_curr = lambda0;

% History storage (can be removed if not used elsewhere, but kept for now)
q_hist = zeros(N_steps, 2);
lambda_hist = zeros(N_steps, 1);
p_rcm_actual_hist = zeros(N_steps, 2);
theta_actual_hist = zeros(N_steps, 1);
error_hist = zeros(N_steps, 3);

%% Simulation Loop
disp('Starting simulation...');

figure;
hold on; grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)');
title('2-DOF RCM Arm Simulation'); 
axis_limit = sum(l_val) * 1.1;
xlim([-axis_limit, axis_limit]);
ylim([-axis_limit, axis_limit]);

% Plot fixed trocar position
h_fixed_trocar = plot(p_trocar_val(1), p_trocar_val(2), 'kx', 'MarkerSize', 12, 'LineWidth', 2);

% Plot fixed desired orientation line
orient_line_len = l_val(2) * 0.75;
h_desired_orientation_line = plot([p_trocar_val(1), p_trocar_val(1) + orient_line_len * cos(theta_d_val)], ...
                                   [p_trocar_val(2), p_trocar_val(2) + orient_line_len * sin(theta_d_val)], ...
                                   'k:', 'LineWidth', 1.5);

% Plot handles (for animation - not in legend)
h_link1 = plot(0,0, 'r-', 'LineWidth', 2);
h_link2 = plot(0,0, 'b-', 'LineWidth', 2);
h_joint1_marker = plot(0,0,'ko','MarkerFaceColor','k','MarkerSize',3);
h_joint2_marker = plot(0,0,'ko','MarkerFaceColor','k','MarkerSize',3);
h_ee_marker = plot(0,0,'ko','MarkerFaceColor','k','MarkerSize',3);

% Plot actual RCM (smaller, empty circle)
h_rcm_actual = plot(0,0, 'mo', 'MarkerSize', 6, 'LineWidth', 1.5, 'MarkerFaceColor', 'none');

% Modified Legend
legend([h_fixed_trocar, h_desired_orientation_line, h_rcm_actual], ...
       {'Fixed Trocar', 'Desired Orientation', 'Actual RCM'}, ...
       'Location', 'southwest');

% Text object for displaying current values on the plot
info_text_x = -axis_limit * 0.95; % Position for the text
info_text_y = axis_limit * 0.9;
h_info_text = text(info_text_x, info_text_y, '', 'VerticalAlignment', 'top', 'FontSize', 9);

for k = 1:N_steps
    time_k = (k-1)*dt;
    % --- State for current iteration ---
    q1_k = q_curr(1);
    q2_k = q_curr(2);
    lambda_k = lambda_curr;

    % --- Forward Kinematics (Numerical) ---
    p1_k = p1_fun(q1_k, l_val(1));
    p2_k = p2_fun(q1_k, q2_k, l_val(1), l_val(2));
    p_rcm_actual_k = p_rcm_fun(q1_k, q2_k, l_val(1), l_val(2), lambda_k);
    theta_actual_k = theta_fun(q1_k, q2_k);

    % --- Calculate Control Inputs (Numerical) ---
    J_ext_k = J_ext_fun(q1_k, q2_k, l_val(1), l_val(2), lambda_k);
    e_ext_k = e_ext_fun(q1_k, q2_k, l_val(1), l_val(2), lambda_k, theta_d_val, p_trocar_val);
    w_k = w_fun(lambda_k);

    % Control Law calculation
    pinv_J_ext_k = pinv(J_ext_k, 1e-6);
    
    term1_k = pinv_J_ext_k * dx_d_val;
    term2_k = pinv_J_ext_k * K_ext * e_ext_k;
    term3_k = (eye(3,3) - pinv_J_ext_k * J_ext_k) * w_k;
    
    ds_cmd_k = term1_k + term2_k + term3_k;

    % --- State Update (Euler Integration) ---
    q_curr = q_curr + ds_cmd_k(1:2) * dt;
    lambda_curr = lambda_curr + ds_cmd_k(3) * dt;
    lambda_curr = max(0, min(1, lambda_curr)); % Constrain lambda

    % --- Update Text on Plot ---
    info_string = sprintf('Time: %5.2f s\nAct. Orient (rad): %6.4f\nTarget Orientation (rad): %6.4f\nAct. RCM: %6.3f, %6.3f\nLambda: %6.3f\nTrocar:%6.4f, %6.4f', ...
                          time_k, theta_actual_k,pi/8, p_rcm_actual_k(1), p_rcm_actual_k(2), lambda_k, p_trocar_val(1), p_trocar_val(2));
    set(h_info_text, 'String', info_string);

    % --- Update Plot ---
    set(h_link1, 'XData', [0, p1_k(1)], 'YData', [0, p1_k(2)]);
    set(h_link2, 'XData', [p1_k(1), p2_k(1)], 'YData', [p1_k(2), p2_k(2)]);
    set(h_joint1_marker, 'XData', 0, 'YData', 0);
    set(h_joint2_marker, 'XData', p1_k(1), 'YData', p1_k(2));
    set(h_ee_marker, 'XData', p2_k(1), 'YData', p2_k(2));
    set(h_rcm_actual, 'XData', p_rcm_actual_k(1), 'YData', p_rcm_actual_k(2));
    
    drawnow;
    pause(0.05); % Slow down for visualization
    
    % Check for convergence
    if norm(e_ext_k) < 1e-4 && norm(ds_cmd_k) < 1e-4
        final_info_string = sprintf('CONVERGED!\nTime: %5.2f s\nAct. Orient (rad): %6.4f\nTarget Orientation (rad): %6.4f\nAct. RCM: %6.3f, %6.3f\nLambda: %6.3f\nTrocar:%6.4f, %6.4f', ...
                                  time_k, theta_actual_k, pi/8, p_rcm_actual_k(1), p_rcm_actual_k(2), lambda_k, p_trocar_val(1), p_trocar_val(2));
        set(h_info_text, 'String', final_info_string, 'Color', 'green');
        disp(['Converged at step ', num2str(k), ' (Time = ', num2str(time_k), ' s)']);
        break;
    end
end

if k == N_steps && ~(norm(e_ext_k) < 1e-4 && norm(ds_cmd_k) < 1e-4)
    final_info_string = sprintf('SIM END (Max Steps)\nTime: %5.2f s\nAct. Orient (rad): %6.4f\nTarget Orientation (rad): %6.4f\nAct. RCM: %6.3f, %6.3f\nLambda: %6.3f\nTrocar:%6.4f, %6.4f', ...
                              time_k, theta_actual_k,pi/8, p_rcm_actual_k(1), p_rcm_actual_k(2), lambda_k, p_trocar_val(1), p_trocar_val(2));
    set(h_info_text, 'String', final_info_string, 'Color', 'red');
    disp('Simulation finished (max steps reached).');
else
    disp('Simulation finished (converged).');
end
