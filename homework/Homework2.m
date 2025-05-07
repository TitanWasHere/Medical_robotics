clear; clc; close all;

% --- SYMBOLIC DEFINITIONS ---
syms q1 q2 q3 q4 l1 l2 l3 l4 real % Added q4, l4
syms dq1 dq2 dq3 dq4 ddq1 ddq2 ddq3 ddq4 real
syms theta_d_sym real
syms p_trocar_x_sym p_trocar_y_sym real

% --- FIXED VALUES & INITIAL CONDITIONS ---
lambda_fixed_val = 0.5;
theta_d_val = pi/8;
% Initial q = [q1,q2,q3,q4], adjust q3,q4 for a reasonable start for 4DOF
q0_val = [pi/4, 0, -pi/16, 0];
% l = [l1,l2,l3,l4]. l1,l2 for shaft, l3,l4 for tool part.
l_val = [1, 1, 0.4, 0.35]; % Ensure l4 > 0

% --- SYMBOLIC KINEMATICS (4-LINK ARM) ---
q_sym = [q1; q2; q3; q4];

p1_sym = [l1 * cos(q1); l1 * sin(q1)];
p2_sym = p1_sym + [l2 * cos(q1+q2); l2 * sin(q1+q2)];
p3_sym = p2_sym + [l3 * cos(q1+q2+q3); l3 * sin(q1+q2+q3)];
p4_sym = p3_sym + [l4 * cos(q1+q2+q3+q4); l4 * sin(q1+q2+q3+q4)]; % New End-Effector

% Task Definition
% 1. RCM on link 2 (x-coordinate)
task1_rcm_x_sym = p1_sym(1) + lambda_fixed_val * (p2_sym(1) - p1_sym(1));
% 2. RCM on link 2 (y-coordinate)
task2_rcm_y_sym = p1_sym(2) + lambda_fixed_val * (p2_sym(2) - p1_sym(2));

% For convenience, group RCM symbolic expression
p_rcm_on_link2_sym = [task1_rcm_x_sym; task2_rcm_y_sym];

% 3. Orientation of vector from Trocar to P4
V_tool_x_sym = p4_sym(1) - p_trocar_x_sym;
V_tool_y_sym = p4_sym(2) - p_trocar_y_sym;
task3_theta_tool_vec_sym = atan2(V_tool_y_sym, V_tool_x_sym);

% 4. Absolute orientation of Link 4
task4_phi_L4_sym = q1 + q2 + q3 + q4;

% --- INITIAL NUMERICAL VALUES ---
p_rcm_init_calc_sym = subs(p_rcm_on_link2_sym, {l1, l2}, {l_val(1), l_val(2)});
p_trocar_val_num = double(subs(p_rcm_init_calc_sym, {q1, q2}, {q0_val(1), q0_val(2)}));

% --- KINEMATIC CONTROL (Symbolic) ---
% Jacobians for each task component
J_task1_rcm_x = jacobian(task1_rcm_x_sym, q_sym);
J_task2_rcm_y = jacobian(task2_rcm_y_sym, q_sym);
J_task3_theta_tool_vec = jacobian(task3_theta_tool_vec_sym, q_sym);
J_task4_phi_L4 = jacobian(task4_phi_L4_sym, q_sym);

% Overall Task Jacobian (4x4)
J_ext_sym = [J_task1_rcm_x; J_task2_rcm_y; J_task3_theta_tool_vec; J_task4_phi_L4];

% Task Errors (Desired - Actual)
e_rcm_x_sym = p_trocar_x_sym - task1_rcm_x_sym;
e_rcm_y_sym = p_trocar_y_sym - task2_rcm_y_sym;

e_theta_tool_raw_sym = theta_d_sym - task3_theta_tool_vec_sym;
e_theta_tool_corr_sym = atan2(sin(e_theta_tool_raw_sym), cos(e_theta_tool_raw_sym));

e_phi_L4_raw_sym = theta_d_sym - task4_phi_L4_sym; % Target for Link 4 orient is also theta_d_sym
e_phi_L4_corr_sym = atan2(sin(e_phi_L4_raw_sym), cos(e_phi_L4_raw_sym));

e_ext_sym = [e_rcm_x_sym; e_rcm_y_sym; e_theta_tool_corr_sym; e_phi_L4_corr_sym]; % 4x1 error

% Gain matrix (4x4)
k_rcm_pos = 7;
k_theta_tool = 5;
k_phi_L4 = 5; % Gain for the new end-effector orientation task
K_ext = diag([k_rcm_pos, k_rcm_pos, k_theta_tool, k_phi_L4]);

% Desired task velocity (all zeros for regulation)
dx_d_val = [0; 0; 0; 0];

% --- CONVERT SYMBOLIC EXPRESSIONS TO MATLAB FUNCTIONS ---
p1_fun = matlabFunction(p1_sym, 'Vars', {q1, l1});
p2_fun = matlabFunction(p2_sym, 'Vars', {q1, q2, l1, l2});
p3_fun = matlabFunction(p3_sym, 'Vars', {q1, q2, q3, l1, l2, l3});
p4_fun = matlabFunction(p4_sym, 'Vars', {q1, q2, q3, q4, l1, l2, l3, l4});

% Functions to get actual task values
task1_rcm_x_fun = matlabFunction(task1_rcm_x_sym, 'Vars', {q1, q2, l1, l2});
task2_rcm_y_fun = matlabFunction(task2_rcm_y_sym, 'Vars', {q1, q2, l1, l2});
task3_theta_tool_vec_fun = matlabFunction(task3_theta_tool_vec_sym, 'Vars', {q1, q2, q3, q4, l1, l2, l3, l4, p_trocar_x_sym, p_trocar_y_sym});
task4_phi_L4_fun = matlabFunction(task4_phi_L4_sym, 'Vars', {q1, q2, q3, q4});

J_ext_fun = matlabFunction(J_ext_sym, 'Vars', {q1, q2, q3, q4, l1, l2, l3, l4, p_trocar_x_sym, p_trocar_y_sym});
e_ext_fun = matlabFunction(e_ext_sym, 'Vars', {q1, q2, q3, q4, l1, l2, l3, l4, theta_d_sym, p_trocar_x_sym, p_trocar_y_sym});


% --- SIMULATION PARAMETERS ---
dt = 0.02;
T_sim = 15; % Increased simulation time
N_steps = round(T_sim/dt);
q_curr = q0_val(:);

% --- SIMULATION LOOP ---
disp('Starting 4-DOF simulation: RCM, Tool Vector Orient, Link 4 Abs Orient...');
figure;
hold on; grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)');
title('4-DOF RCM Arm: Tool Vector & Link 4 Orientation (Fixed Lambda)');
axis_limit = sum(l_val) * 1.05;
xlim([-axis_limit, axis_limit]); ylim([-axis_limit, axis_limit]);

h_fixed_trocar = plot(p_trocar_val_num(1), p_trocar_val_num(2), 'kx', 'MarkerSize', 12, 'LineWidth', 2);
orient_line_len = (l_val(3)+l_val(4)) * 1.1;
h_desired_orientation_line = plot([p_trocar_val_num(1), p_trocar_val_num(1) + orient_line_len * cos(theta_d_val)], ...
                                   [p_trocar_val_num(2), p_trocar_val_num(2) + orient_line_len * sin(theta_d_val)], ...
                                   'k:', 'LineWidth', 1.5);
h_link1 = plot(0,0, 'r-', 'LineWidth', 2);
h_link2 = plot(0,0, 'b-', 'LineWidth', 2);
h_link3 = plot(0,0, 'g-', 'LineWidth', 2);
h_link4 = plot(0,0, 'm-', 'LineWidth', 2); % Link 4 in magenta

h_joint0_marker = plot(0,0,'k^','MarkerFaceColor','k','MarkerSize',8); % Base
h_joint1_marker = plot(0,0,'ko','MarkerFaceColor','k','MarkerSize',6); % End of L1
h_joint2_marker = plot(0,0,'ko','MarkerFaceColor','k','MarkerSize',6); % End of L2
h_joint3_marker = plot(0,0,'ko','MarkerFaceColor','k','MarkerSize',6); % End of L3
h_ee_marker = plot(0,0,'ks','MarkerFaceColor','k','MarkerSize',6);     % End of L4 (P4)

h_rcm_actual = plot(0,0, 'co', 'MarkerSize', 7, 'LineWidth', 1.5, 'MarkerFaceColor', 'none'); % Cyan RCM circle
%h_actual_tool_vector = plot(0,0, 'c--', 'LineWidth', 1.5); % Cyan dashed: RCM to P4

%legend([h_fixed_trocar, h_desired_orientation_line, h_rcm_actual, h_actual_tool_vector, h_link4], ...
%       {'Fixed Trocar', 'Desired Orientation (Tool & Link4)', 'Actual RCM (on Link 2)', 'Actual Tool Vector (RCM to P4)', 'Link 4'}, ...
%       'Location', 'southwest', 'FontSize', 7);

legend([h_fixed_trocar, h_desired_orientation_line, h_rcm_actual, h_link4], ...
       {'Fixed Trocar', 'Desired Orientation (Tool & Link4)', 'Actual RCM (on Link 2)', 'Link 4'}, ...
       'Location', 'southwest', 'FontSize', 7);

info_text_x = -axis_limit * 0.95; info_text_y = axis_limit * 0.9;
h_info_text = text(info_text_x, info_text_y, '', 'VerticalAlignment', 'top', 'FontSize', 8, 'FontName', 'Consolas');

for k_sim = 1:N_steps
    time_k = (k_sim-1)*dt;
    q1_k = q_curr(1); q2_k = q_curr(2); q3_k = q_curr(3); q4_k = q_curr(4);

    % --- Forward Kinematics ---
    p1_k = p1_fun(q1_k, l_val(1));
    p2_k = p2_fun(q1_k, q2_k, l_val(1), l_val(2));
    p3_k = p3_fun(q1_k, q2_k, q3_k, l_val(1), l_val(2), l_val(3));
    p4_k = p4_fun(q1_k, q2_k, q3_k, q4_k, l_val(1), l_val(2), l_val(3), l_val(4));

    % --- Actual Task Values ---
    rcm_actual_x_k = task1_rcm_x_fun(q1_k, q2_k, l_val(1), l_val(2));
    rcm_actual_y_k = task2_rcm_y_fun(q1_k, q2_k, l_val(1), l_val(2));
    theta_tool_actual_k = task3_theta_tool_vec_fun(q1_k, q2_k, q3_k, q4_k, l_val(1),l_val(2),l_val(3),l_val(4), p_trocar_val_num(1), p_trocar_val_num(2));
    phi_L4_actual_k = task4_phi_L4_fun(q1_k, q2_k, q3_k, q4_k);

    % --- Control Inputs ---
    J_ext_k = J_ext_fun(q1_k,q2_k,q3_k,q4_k, l_val(1),l_val(2),l_val(3),l_val(4), p_trocar_val_num(1), p_trocar_val_num(2));
    e_ext_k = e_ext_fun(q1_k,q2_k,q3_k,q4_k, l_val(1),l_val(2),l_val(3),l_val(4), theta_d_val, p_trocar_val_num(1), p_trocar_val_num(2));

    pinv_J_ext_k = pinv(J_ext_k, 1e-6); %pinv for 4x4 is inv if non-singular
    ds_cmd_k = pinv_J_ext_k * (dx_d_val + K_ext * e_ext_k); % dq_cmd
    q_curr = q_curr + ds_cmd_k * dt;

    % --- Update Text & Plot ---
    info_string = sprintf([...
        'Time: %5.2f s\n' ...
        'Tgt. Orient (Tool & L4): %6.4f rad\n'...
        'Act. Tool Orient (RCM-P4): %6.4f rad\n' ...
        'Act. Link4 Abs Orient: %6.4f rad\n' ...
        'Act. RCM (L2): (%6.3f, %6.3f)\n' ...
        'Tgt. RCM (Trocar):(%6.3f, %6.3f)\n' ...
        'Fixed Lambda: %4.2f\n' ...
        'q = [%5.2f,%5.2f,%5.2f,%5.2f] rad'], ...
        time_k, theta_d_val, ...
        theta_tool_actual_k, phi_L4_actual_k, ...
        rcm_actual_x_k, rcm_actual_y_k, ...
        p_trocar_val_num(1), p_trocar_val_num(2), ...
        lambda_fixed_val, ...
        q_curr(1), q_curr(2), q_curr(3), q_curr(4));
    set(h_info_text, 'String', info_string);

    set(h_link1, 'XData', [0, p1_k(1)], 'YData', [0, p1_k(2)]);
    set(h_link2, 'XData', [p1_k(1), p2_k(1)], 'YData', [p1_k(2), p2_k(2)]);
    set(h_link3, 'XData', [p2_k(1), p3_k(1)], 'YData', [p2_k(2), p3_k(2)]);
    set(h_link4, 'XData', [p3_k(1), p4_k(1)], 'YData', [p3_k(2), p4_k(2)]); % Plot Link 4

    set(h_joint0_marker, 'XData', 0, 'YData', 0);
    set(h_joint1_marker, 'XData', p1_k(1), 'YData', p1_k(2));
    set(h_joint2_marker, 'XData', p2_k(1), 'YData', p2_k(2));
    set(h_joint3_marker, 'XData', p3_k(1), 'YData', p3_k(2)); % Marker at end of Link 3
    set(h_ee_marker, 'XData', p4_k(1), 'YData', p4_k(2));     % Marker at end of Link 4

    set(h_rcm_actual, 'XData', rcm_actual_x_k, 'YData', rcm_actual_y_k);
    %set(h_actual_tool_vector, 'XData', [rcm_actual_x_k, p4_k(1)], 'YData', [rcm_actual_y_k, p4_k(2)]);


    drawnow;
    pause(0.05);

    % Convergence Check (check errors for each task component)
    rcm_pos_error_norm = norm([e_ext_k(1), e_ext_k(2)]);
    tool_orient_error_abs = abs(e_ext_k(3));
    L4_orient_error_abs = abs(e_ext_k(4));

    if rcm_pos_error_norm < 1e-3 && tool_orient_error_abs < 1e-3 && L4_orient_error_abs < 1e-3 && norm(ds_cmd_k) < 1e-3
        % (Update convergence message for 4 tasks)
        final_info_string = sprintf([...
            'CONVERGED!\n' ...
            'Time: %5.2f s\n' ...
            'Tgt. Orient (Tool & L4): %6.4f rad\n'...
            'Act. Tool Orient (RCM-P4): %6.4f rad\n' ...
            'Act. Link4 Abs Orient: %6.4f rad\n' ...
            'Act. RCM (L2): (%6.3f, %6.3f)\n' ...
            'Tgt. RCM (Trocar):(%6.3f, %6.3f)\n' ...
            'Fixed Lambda: %4.2f\n' ...
            'q = [%5.2f,%5.2f,%5.2f,%5.2f] rad'], ...
            time_k, theta_d_val, ...
            theta_tool_actual_k, phi_L4_actual_k, ...
            rcm_actual_x_k, rcm_actual_y_k, ...
            p_trocar_val_num(1), p_trocar_val_num(2), ...
            lambda_fixed_val, ...
            q_curr(1), q_curr(2), q_curr(3), q_curr(4));
        set(h_info_text, 'String', final_info_string, 'Color', 'blue');
        disp(['Converged at step ', num2str(k_sim), ' (Time = ', num2str(time_k), ' s)']);
        break;
    end
end
% (Update sim end message for 4 tasks)
if k_sim == N_steps && ~(rcm_pos_error_norm < 1e-3 && tool_orient_error_abs < 1e-3 && L4_orient_error_abs < 1e-3 && norm(ds_cmd_k) < 1e-3)
    final_info_string = sprintf([...
        'SIM END (Max Steps)\n' ...
        'Time: %5.2f s\n' ...
        'Tgt. Orient (Tool & L4): %6.4f rad\n'...
        'Act. Tool Orient (RCM-P4): %6.4f rad\n' ...
        'Act. Link4 Abs Orient: %6.4f rad\n' ...
        'Act. RCM (L2): (%6.3f, %6.3f)\n' ...
        'Tgt. RCM (Trocar):(%6.3f, %6.3f)\n' ...
        'Fixed Lambda: %4.2f\n' ...
        'q = [%5.2f,%5.2f,%5.2f,%5.2f] rad'], ...
        time_k, theta_d_val, ...
        theta_tool_actual_k, phi_L4_actual_k, ...
        rcm_actual_x_k, rcm_actual_y_k, ...
        p_trocar_val_num(1), p_trocar_val_num(2), ...
        lambda_fixed_val, ...
        q_curr(1), q_curr(2), q_curr(3), q_curr(4));
    set(h_info_text, 'String', final_info_string, 'Color', 'red');
    disp('Simulation finished (max steps reached).');
elseif ~(k_sim == N_steps) % Converged
     disp('Simulation finished (converged).');
end