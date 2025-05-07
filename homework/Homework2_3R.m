clear; clc; close all;

% --- SYMBOLIC DEFINITIONS ---
syms q1 q2 q3 l1 l2 l3 real
syms dq1 dq2 dq3 ddq1 ddq2 ddq3 real
syms theta_d_sym real
syms p_trocar_x_sym p_trocar_y_sym real

lambda_fixed_val = 0.5;
theta_d_val = pi/8;
q0_val = [pi/4, 0, -pi/16]; 
l_val = [1, 1, 0.75];       

% --- SYMBOLIC KINEMATICS ---
q_sym = [q1; q2; q3];

p1_sym = [l1 * cos(q1); l1 * sin(q1)];
p2_sym = p1_sym + [l2 * cos(q1+q2); l2 * sin(q1+q2)];
p3_sym = p2_sym + [l3 * cos(q1+q2+q3); l3 * sin(q1+q2+q3)];

% Task 1 & 2: RCM point on the *second link*
p_rcm_on_link2_sym = p1_sym + lambda_fixed_val * (p2_sym - p1_sym);
J_rcm_on_link2_sym = jacobian(p_rcm_on_link2_sym, q_sym); 

% Task 3: Orientation of the vector from Trocar to P3

V_tool_x_sym = p3_sym(1) - p_trocar_x_sym;
V_tool_y_sym = p3_sym(2) - p_trocar_y_sym;
theta_tool_from_trocar_sym = atan2(V_tool_y_sym, V_tool_x_sym);
J_theta_tool_from_trocar_sym = jacobian(theta_tool_from_trocar_sym, q_sym); 

% --- INITIAL NUMERICAL VALUES ---
p_rcm_init_calc_sym = subs(p_rcm_on_link2_sym, {l1, l2}, {l_val(1), l_val(2)});
p_trocar_val_num = double(subs(p_rcm_init_calc_sym, {q1, q2}, {q0_val(1), q0_val(2)})); 

% --- KINEMATIC CONTROL (Symbolic) ---
J_ext_sym = [J_theta_tool_from_trocar_sym; J_rcm_on_link2_sym];

% Task Errors
% Angular error for theta_tool (normalized)
e_theta_raw_sym = theta_d_sym - theta_tool_from_trocar_sym;
e_theta_corr_sym = atan2(sin(e_theta_raw_sym), cos(e_theta_raw_sym));

% RCM error (vector from actual RCM to target trocar)
p_trocar_target_as_sym_vec = [p_trocar_x_sym; p_trocar_y_sym]; % Use symbolic trocar for error definition
e_rcm_sym = p_trocar_target_as_sym_vec - p_rcm_on_link2_sym;
e_ext_sym = [e_theta_corr_sym ; e_rcm_sym]; % 3x1 error vector

% Gain matrix
k_rcm = 7; % Increased RCM gain slightly
k_theta = 5; % Increased Theta gain slightly
K_ext = diag([k_theta, k_rcm, k_rcm]);

% Desired task velocity
dx_d_val = [0; 0; 0]; % For [dtheta_tool_dt; dpx_rcm_dt; dpy_rcm_dt]

% --- CONVERT SYMBOLIC EXPRESSIONS TO MATLAB FUNCTIONS ---
p1_fun = matlabFunction(p1_sym, 'Vars', {q1, l1});
p2_fun = matlabFunction(p2_sym, 'Vars', {q1, q2, l1, l2});
p3_fun = matlabFunction(p3_sym, 'Vars', {q1, q2, q3, l1, l2, l3});
p_rcm_calc_fun = matlabFunction(p_rcm_on_link2_sym, 'Vars', {q1, q2, l1, l2});
theta_tool_calc_fun = matlabFunction(theta_tool_from_trocar_sym, 'Vars', {q1, q2, q3, l1, l2, l3, p_trocar_x_sym, p_trocar_y_sym});

% J_ext_fun will depend on q1,q2,q3, l1,l2,l3, p_trocar_x_sym, p_trocar_y_sym
J_ext_fun = matlabFunction(J_ext_sym, 'Vars', {q1, q2, q3, l1, l2, l3, p_trocar_x_sym, p_trocar_y_sym});

% e_ext_fun will depend on q1,q2,q3, l1,l2,l3, theta_d_sym, p_trocar_x_sym, p_trocar_y_sym
e_ext_fun = matlabFunction(e_ext_sym, 'Vars', {q1, q2, q3, l1, l2, l3, theta_d_sym, p_trocar_x_sym, p_trocar_y_sym});

% --- SIMULATION PARAMETERS ---
dt = 0.02;
T_sim = 12; % Slightly longer simulation time
N_steps = round(T_sim/dt);
q_curr = q0_val(:);

% --- SIMULATION LOOP ---
disp('Starting 3-DOF simulation with fixed lambda and corrected orientation task...');
figure;
hold on; grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)');
title('3-DOF RCM Arm: Tool Orientation from Trocar (Fixed Lambda)');
axis_limit = sum(l_val) * 1.05;
xlim([-axis_limit, axis_limit]); ylim([-axis_limit, axis_limit]);

orient_line_len = l_val(3) * 1.8; 
h_desired_orientation_line = plot([p_trocar_val_num(1), p_trocar_val_num(1) + orient_line_len * cos(theta_d_val)], ...
                                   [p_trocar_val_num(2), p_trocar_val_num(2) + orient_line_len * sin(theta_d_val)], ...
                                   'k:', 'LineWidth', 1.5);
h_link1 = plot(0,0, 'r-', 'LineWidth', 2);
h_link2 = plot(0,0, 'b-', 'LineWidth', 2);
h_link3 = plot(0,0, 'g-', 'LineWidth', 2);
h_joint0_marker = plot(0,0,'k^','MarkerFaceColor','k','MarkerSize',2);
h_joint1_marker = plot(0,0,'ko','MarkerFaceColor','k','MarkerSize',2);
h_joint2_marker = plot(0,0,'ko','MarkerFaceColor','k','MarkerSize',2);
h_ee_marker = plot(0,0,'ks','MarkerFaceColor','k','MarkerSize',2);
h_rcm_actual = plot(0,0, 'mo', 'MarkerSize', 6, 'LineWidth', 1.5, 'MarkerFaceColor', 'none');
h_fixed_trocar = plot(p_trocar_val_num(1), p_trocar_val_num(2), 'kx', 'MarkerSize', 12, 'LineWidth', 2);

% New: Plot for the actual tool vector from RCM to P3
%h_actual_tool_vector = plot(0,0, 'c--', 'LineWidth', 1.5); % Cyan dashed line

%legend([h_fixed_trocar, h_desired_orientation_line, h_rcm_actual, h_actual_tool_vector], ...
%       {'Fixed Trocar', 'Desired Tool Orientation', 'Actual RCM (on Link 2)', 'Actual Tool Vector (RCM to P3)'}, ...
%       'Location', 'southwest', 'FontSize', 8);

legend([h_fixed_trocar, h_desired_orientation_line, h_rcm_actual], ...
       {'Fixed Trocar', 'Desired Tool Orientation', 'Actual RCM (on Link 2)'}, ...
       'Location', 'southwest', 'FontSize', 8);

info_text_x = -axis_limit * 0.95; info_text_y = axis_limit * 0.9;
h_info_text = text(info_text_x, info_text_y, '', 'VerticalAlignment', 'top', 'FontSize', 9, 'FontName', 'Consolas');

for k_sim = 1:N_steps
    time_k = (k_sim-1)*dt;
    q1_k = q_curr(1); q2_k = q_curr(2); q3_k = q_curr(3);

    p1_k = p1_fun(q1_k, l_val(1));
    p2_k = p2_fun(q1_k, q2_k, l_val(1), l_val(2));
    p3_k = p3_fun(q1_k, q2_k, q3_k, l_val(1), l_val(2), l_val(3));
    p_rcm_actual_k = p_rcm_calc_fun(q1_k, q2_k, l_val(1), l_val(2));
    % Actual tool orientation from trocar (using numerical trocar position)
    theta_tool_actual_k = theta_tool_calc_fun(q1_k, q2_k, q3_k, l_val(1), l_val(2), l_val(3), p_trocar_val_num(1), p_trocar_val_num(2));

    % --- Control Inputs ---
    % Pass numerical trocar_x, trocar_y to functions expecting symbolic placeholders
    J_ext_k = J_ext_fun(q1_k, q2_k, q3_k, l_val(1), l_val(2), l_val(3), p_trocar_val_num(1), p_trocar_val_num(2));
    e_ext_k = e_ext_fun(q1_k, q2_k, q3_k, l_val(1), l_val(2), l_val(3), theta_d_val, p_trocar_val_num(1), p_trocar_val_num(2));

    pinv_J_ext_k = pinv(J_ext_k, 1e-6);
    ds_cmd_k = pinv_J_ext_k * (dx_d_val + K_ext * e_ext_k);
    q_curr = q_curr + ds_cmd_k * dt;

    % --- Update Text & Plot ---
    info_string = sprintf([...
        'Time: %5.2f s\n' ...
        'Act. Tool Orient: %6.4f rad\n' ...
        'Tgt. Tool Orient: %6.4f rad\n' ...
        'Act. RCM (L2): (%6.3f, %6.3f)\n' ...
        'Tgt. RCM (Trocar):(%6.3f, %6.3f)\n' ...
        'Fixed Lambda: %4.2f\n' ...
        'q = [%5.2f, %5.2f, %5.2f] rad'], ...
        time_k, theta_tool_actual_k, theta_d_val, ...
        p_rcm_actual_k(1), p_rcm_actual_k(2), ...
        p_trocar_val_num(1), p_trocar_val_num(2), ...
        lambda_fixed_val, ...
        q_curr(1), q_curr(2), q_curr(3));
    set(h_info_text, 'String', info_string);

    set(h_link1, 'XData', [0, p1_k(1)], 'YData', [0, p1_k(2)]);
    set(h_link2, 'XData', [p1_k(1), p2_k(1)], 'YData', [p1_k(2), p2_k(2)]);
    set(h_link3, 'XData', [p2_k(1), p3_k(1)], 'YData', [p2_k(2), p3_k(2)]);
    set(h_joint0_marker, 'XData', 0, 'YData', 0);
    set(h_joint1_marker, 'XData', p1_k(1), 'YData', p1_k(2));
    set(h_joint2_marker, 'XData', p2_k(1), 'YData', p2_k(2));
    set(h_ee_marker, 'XData', p3_k(1), 'YData', p3_k(2));
    set(h_rcm_actual, 'XData', p_rcm_actual_k(1), 'YData', p_rcm_actual_k(2));
    % Update actual tool vector plot
    %set(h_actual_tool_vector, 'XData', [p_rcm_actual_k(1), p3_k(1)], 'YData', [p_rcm_actual_k(2), p3_k(2)]);


    drawnow;
    pause(0.05);

    if norm(e_ext_k(2:3)) < 1e-3 && abs(e_ext_k(1)) < 1e-3 && norm(ds_cmd_k) < 1e-3 % Stricter error check
        % ... (convergence message update) ...
        final_info_string = sprintf([...
            'CONVERGED!\n' ...
            'Time: %5.2f s\n' ...
            'Act. Tool Orient: %6.4f rad\n' ...
            'Tgt. Tool Orient: %6.4f rad\n' ...
            'Act. RCM (L2): (%6.3f, %6.3f)\n' ...
            'Tgt. RCM (Trocar):(%6.3f, %6.3f)\n' ...
            'Fixed Lambda: %4.2f\n' ...
            'q = [%5.2f, %5.2f, %5.2f] rad'], ...
            time_k, theta_tool_actual_k, theta_d_val, ...
            p_rcm_actual_k(1), p_rcm_actual_k(2), ...
            p_trocar_val_num(1), p_trocar_val_num(2), ...
            lambda_fixed_val, ...
            q_curr(1), q_curr(2), q_curr(3));
        set(h_info_text, 'String', final_info_string, 'Color', 'green');
        disp(['Converged at step ', num2str(k_sim), ' (Time = ', num2str(time_k), ' s)']);
        break;
    end
end
% ... (simulation end message update, similar to above) ...
if k_sim == N_steps && ~(norm(e_ext_k(2:3)) < 1e-3 && abs(e_ext_k(1)) < 1e-3 && norm(ds_cmd_k) < 1e-3)
    final_info_string = sprintf([...
        'SIM END (Max Steps)\n' ...
         'Time: %5.2f s\n' ...
        'Act. Tool Orient: %6.4f rad\n' ...
        'Tgt. Tool Orient: %6.4f rad\n' ...
        'Act. RCM (L2): (%6.3f, %6.3f)\n' ...
        'Tgt. RCM (Trocar):(%6.3f, %6.3f)\n' ...
        'Fixed Lambda: %4.2f\n' ...
        'q = [%5.2f, %5.2f, %5.2f] rad'], ...
        time_k, theta_tool_actual_k, theta_d_val, ...
        p_rcm_actual_k(1), p_rcm_actual_k(2), ...
        p_trocar_val_num(1), p_trocar_val_num(2), ...
        lambda_fixed_val, ...
        q_curr(1), q_curr(2), q_curr(3));
    set(h_info_text, 'String', final_info_string, 'Color', 'red');
    disp('Simulation finished (max steps reached).');
elseif ~(k_sim == N_steps) % Converged
     disp('Simulation finished (converged).');
end