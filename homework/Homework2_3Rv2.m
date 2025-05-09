clear; clc; close all;

syms q1 q2 q3 l1 l2 l3 real
syms dq1 dq2 dq3 ddq1 ddq2 ddq3 real
syms theta_d_sym real
syms p_trocar_x_sym p_trocar_y_sym real 

%% Definition of values from user
lambda_val = 0.5; 
theta_d_val = pi/8; 
q_init = [0, pi/4, 0]; 
l_val = [1,1,1]; 

%% Symbolic values
q_sym = [q1;q2;q3]; 
dq_sym = [dq1;dq2;dq3];

% Kinematic chain
p_joint1_sym = [l1 * cos(q1) ; l1 * sin(q1)]; 
p_joint2_sym = p_joint1_sym + [l2 * cos(q1+q2) ; l2 * sin(q1+q2)]; 
p_ee_sym = p_joint2_sym + [l3 * cos(q1+q2+q3) ; l3 * sin(q1+q2+q3)]; 

% Task 1: End-effector (link 3) orientation
theta_sym = q1 + q2 + q3;
J_theta_sym = jacobian(theta_sym, q_sym); 

% Task 2: RCM position
vector_link3_sym = [l3 * cos(q1+q2+q3) ; l3 * sin(q1+q2+q3)];
p_rcm_sym = p_joint2_sym + lambda_val * vector_link3_sym; 
J_rcm_q_sym = jacobian(p_rcm_sym, q_sym); 

%% Initial values (Numerical)
p_trocar_val = double(subs(p_rcm_sym, {q1,q2,q3,l1,l2,l3}, {q_init(1), q_init(2), q_init(3), l_val(1), l_val(2), l_val(3)}));

%% Kinematic control (Symbolic)
% Combined Jacobian: 1st row for orientation, next 2 for RCM position
J_ext_sym = [J_theta_sym; J_rcm_q_sym]; 

p_trocar_target_sym = [p_trocar_x_sym; p_trocar_y_sym]; 
e_theta_sym = theta_d_sym - theta_sym;
e_rcm_sym = p_trocar_target_sym - p_rcm_sym;
e_ext_sym = [e_theta_sym ; e_rcm_sym]; 

% Gain matrices
k_rcm = 5;  
k_theta = 3; 
K_ext = diag([k_theta, k_rcm, k_rcm]); 

% Task velocity (desired) - all zero for regulation task
dtheta_d_val_num = 0;
dp_rcm_d_val_num = [0;0]; 
dx_d_val = [dtheta_d_val_num ; dp_rcm_d_val_num]; 

%% Convert symbolic expressions to MATLAB functions for speed
p_joint1_fun = matlabFunction(p_joint1_sym, 'Vars', {q1, l1});
p_joint2_fun = matlabFunction(p_joint2_sym, 'Vars', {q1, q2, l1, l2});
p_ee_fun = matlabFunction(p_ee_sym, 'Vars', {q1, q2, q3, l1, l2, l3});
p_rcm_fun = matlabFunction(p_rcm_sym, 'Vars', {q1, q2, q3, l1, l2, l3});
theta_fun = matlabFunction(theta_sym, 'Vars', {q1, q2, q3});
J_ext_fun = matlabFunction(J_ext_sym, 'Vars', {q1, q2, q3, l1, l2, l3});
e_ext_fun = matlabFunction(e_ext_sym, 'Vars', {q1, q2, q3, l1, l2, l3, theta_d_sym, p_trocar_target_sym});

%% Simulation Parameters
dt = 0.02;       
T_sim = 15;   
N_steps = round(T_sim/dt);

% Initial conditions for simulation state
q_curr = q_init(:);


%% Simulation Loop
disp('Starting simulation...');

figure;
hold on; grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)');
title('3-DOF Arm with fixed lambda'); 
axis_limit = sum(l_val) * 1.2;
xlim([-axis_limit, axis_limit]);
ylim([-axis_limit, axis_limit]);

% Plot fixed trocar position (this is where RCM must stay)
h_fixed_trocar_rcm = plot(p_trocar_val(1), p_trocar_val(2), 'kx', 'MarkerSize', 12, 'LineWidth', 2);

% Plot fixed desired orientation line (originating from the fixed RCM/Trocar point)
orient_line_len = l_val(3) * (1-lambda_val) * 1.5; % Length of line relative to segment from RCM to EE
h_desired_orientation_line = plot([p_trocar_val(1), p_trocar_val(1) + orient_line_len * cos(theta_d_val)], ...
                                   [p_trocar_val(2), p_trocar_val(2) + orient_line_len * sin(theta_d_val)], ...
                                   'k:', 'LineWidth', 1.5);

% Plot handles (for animation)
h_link1 = plot(0,0, 'r-', 'LineWidth', 2);
h_link2 = plot(0,0, 'b-', 'LineWidth', 2);
h_link3 = plot(0,0, 'g-', 'LineWidth', 2); % Changed to green for visibility

h_base_marker = plot(0,0,'k^','MarkerFaceColor','k','MarkerSize',3); % Base at origin
h_joint1_marker = plot(0,0,'ko','MarkerFaceColor','k','MarkerSize',2); % Joint 2
h_joint2_marker = plot(0,0,'ko','MarkerFaceColor','k','MarkerSize',2); % Joint 3
h_ee_marker = plot(0,0,'ko','MarkerFaceColor','k','MarkerSize',1); % End-effector

% Plot actual RCM (should stay on top of h_fixed_trocar_rcm)
h_rcm_actual_plot = plot(0,0, 'mo', 'MarkerSize', 8, 'LineWidth', 1.5, 'MarkerFaceColor', 'none');

legend([h_fixed_trocar_rcm, h_desired_orientation_line, h_rcm_actual_plot, h_link1, h_link2, h_link3], ...
       {'Fixed Trocar (RCM Target)', 'Desired EE Orientation', 'Actual RCM', 'Link 1', 'Link 2', 'Link 3'}, ...
       'Location', 'southwest');

info_text_x = -axis_limit * 0.95; 
info_text_y = axis_limit * 0.9;
h_info_text = text(info_text_x, info_text_y, '', 'VerticalAlignment', 'top', 'FontSize', 9);

for k_sim = 1:N_steps
    time_k = (k_sim-1)*dt;
    % --- State for current iteration ---
    q1_k = q_curr(1);
    q2_k = q_curr(2);
    q3_k = q_curr(3);

    % --- Forward Kinematics (Numerical) ---
    p_j1_k = p_joint1_fun(q1_k, l_val(1));
    p_j2_k = p_joint2_fun(q1_k, q2_k, l_val(1), l_val(2));
    p_ee_k = p_ee_fun(q1_k, q2_k, q3_k, l_val(1), l_val(2), l_val(3));
    
    p_rcm_actual_k = p_rcm_fun(q1_k, q2_k, q3_k, l_val(1), l_val(2), l_val(3));
    theta_actual_k = theta_fun(q1_k, q2_k, q3_k);

    % --- Calculate Control Inputs (Numerical) ---
    J_ext_k_val = J_ext_fun(q1_k, q2_k, q3_k, l_val(1), l_val(2), l_val(3));
    e_ext_k_val = e_ext_fun(q1_k, q2_k, q3_k, l_val(1), l_val(2), l_val(3), theta_d_val, p_trocar_val); 
    
    % Control Law
    pinv_J_ext_k = pinv(J_ext_k_val, 1e-6); 
    
    dq_cmd_k = pinv_J_ext_k * (dx_d_val + K_ext * e_ext_k_val); % dq_cmd_k is 3x1

    % --- State Update (Euler Integration) ---
    q_curr = q_curr + dq_cmd_k * dt;

    % --- Update Text on Plot ---
    info_string = sprintf(['Time: %5.2f s\n', ...
                           'Act. Orient (rad): %6.4f\nTar. Orient (rad): %6.4f\n', ...
                           'Act. RCM: [%6.3f, %6.3f]\nTar. RCM: [%6.3f, %6.3f]\n', ...
                           'Lambda (const): %.2f\n'], ...
                          time_k, theta_actual_k, theta_d_val, ...
                          p_rcm_actual_k(1), p_rcm_actual_k(2), p_trocar_val(1), p_trocar_val(2), ...
                          lambda_val);
    set(h_info_text, 'String', info_string);

    % --- Update Plot ---
    set(h_link1, 'XData', [0, p_j1_k(1)], 'YData', [0, p_j1_k(2)]);
    set(h_link2, 'XData', [p_j1_k(1), p_j2_k(1)], 'YData', [p_j1_k(2), p_j2_k(2)]);
    set(h_link3, 'XData', [p_j2_k(1), p_ee_k(1)], 'YData', [p_j2_k(2), p_ee_k(2)]);
    
    set(h_base_marker, 'XData', 0, 'YData', 0); 
    set(h_joint1_marker, 'XData', p_j1_k(1), 'YData', p_j1_k(2)); 
    set(h_joint2_marker, 'XData', p_j2_k(1), 'YData', p_j2_k(2)); 
    set(h_ee_marker, 'XData', p_ee_k(1), 'YData', p_ee_k(2));     
    set(h_rcm_actual_plot, 'XData', p_rcm_actual_k(1), 'YData', p_rcm_actual_k(2));
    
    drawnow;
    pause(0.01); 
    
    % Check for convergence
    if norm(e_ext_k_val) < 1e-4 && norm(dq_cmd_k) < 1e-4 
        final_info_string = sprintf(['CONVERGED!\nTime: %5.2f s\n', ...
                                   'Act. Orient (rad): %6.4f\nTar. Orient (rad): %6.4f\n', ...
                                   'Act. RCM: [%6.3f, %6.3f]\nTar. RCM: [%6.3f, %6.3f]\n', ...
                                   'Lambda (const): %.2f\n'], ...
                                  time_k, theta_actual_k, theta_d_val, ...
                                  p_rcm_actual_k(1), p_rcm_actual_k(2), p_trocar_val(1), p_trocar_val(2), ...
                                  lambda_val);
        set(h_info_text, 'String', final_info_string, 'Color', 'blue');
        disp(['Converged at step ', num2str(k_sim), ' (Time = ', num2str(time_k), ' s)']);
        break;
    end
end

if k_sim == N_steps && ~(norm(e_ext_k_val) < 1e-4 && norm(dq_cmd_k) < 1e-4)
    final_info_string = sprintf(['SIM END (Max Steps)\nTime: %5.2f s\n', ...
                               'Act. Orient (rad): %6.4f\nTar. Orient (rad): %6.4f\n', ...
                               'Act. RCM: [%6.3f, %6.3f]\nTar. RCM: [%6.3f, %6.3f]\n', ...
                               'Lambda (const): %.2f\n'], ...
                              time_k, theta_actual_k, theta_d_val, ...
                              p_rcm_actual_k(1), p_rcm_actual_k(2), p_trocar_val(1), p_trocar_val(2), ...
                              lambda_val);
    set(h_info_text, 'String', final_info_string, 'Color', 'red');
    disp('Simulation finished (max steps reached).');
else
    disp('Simulation finished.');
end