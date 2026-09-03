% 6-DOF flight control panel prototype: per-axis LQR on [angle; rate]
% driving a 4-thruster RCS allocation matrix, plus a suicide-burn trigger
% for the main engine.
clear; clc; close all;

% --- Vehicle and engine parameters ---
m0 = 27.674; F_main = 600; g = 9.81;
m_dot_main = 0.734; m_dot_rcs = 0.015;
h0 = 10; I_rocket = 1.5; L_rcs = 0.5; F_rcs = 30;
t_delay_main = 0.300; t_delay_rcs = 0.060;

% Control allocation matrix and LQR gain
M = [ 1  1  1;
     -1  1 -1;
      1 -1 -1;
     -1 -1  1];
A = [0 1; 0 0]; B = [0; (L_rcs/I_rocket)];
Q = diag([5000 400]); R = 0.5;
K = lqr(A,B,Q,R);

% --- Simulation setup ---
dt = 0.001; t = 0:dt:5;
n = length(t);
h = zeros(1,n); v = zeros(1,n); mass = zeros(1,n); accel = zeros(1,n);
phi = zeros(1,n); theta = zeros(1,n); psi = zeros(1,n);
p = zeros(1,n); q = zeros(1,n); r = zeros(1,n);
thrust_main = false(1,n); rcs_states = zeros(4, n);
gas_main = zeros(1,n); gas_rcs_total = zeros(1,n);

h(1) = h0; mass(1) = m0;
phi(1) = deg2rad(10); theta(1) = deg2rad(12); psi(1) = deg2rad(10);
open_cmd_t = -1; close_cmd_t = -1; landed = false; idx_land = 1;

% --- Simulation loop ---
for i = 2:n
    if ~landed
        m_now = mass(i-1);

        % Suicide-burn decision
        if open_cmd_t == -1
            v_d = abs(v(i-1)) + (g * t_delay_main);
            h_d = h(i-1) - (abs(v(i-1)) * t_delay_main) - (0.5 * g * t_delay_main^2);
            a_predict = (F_main / m_now) - g;
            if h_d <= (v_d^2 / (2 * a_predict)), open_cmd_t = t(i); end
        elseif close_cmd_t == -1
            a_predict = (F_main / m_now) - g;
            if abs(v(i-1)) <= (a_predict * t_delay_main), close_cmd_t = t(i); end
        end

        % LQR control (per-axis)
        u_roll  = -K * [phi(i-1); p(i-1)];
        u_pitch = -K * [theta(i-1); q(i-1)];
        u_yaw   = -K * [psi(i-1); r(i-1)];
        u_cmd   = [u_roll; u_pitch; u_yaw];

        valve_cmd = M * u_cmd;
        for k = 1:4
            if valve_cmd(k) > 0.05, rcs_states(k,i) = 1; end
        end

        % Physics
        is_burning = (open_cmd_t ~= -1 && t(i) >= open_cmd_t + t_delay_main && ...
                     (close_cmd_t == -1 || t(i) < close_cmd_t + t_delay_main));
        thrust_main(i) = is_burning;
        accel(i) = (is_burning * F_main / m_now) - g;

        delay_rcs = round(t_delay_rcs/dt);
        if i > delay_rcs
            act = rcs_states(:, i-delay_rcs);
            tau = [ (act(1)-act(2)+act(3)-act(4)), (act(1)+act(2)-act(3)-act(4)), (act(1)-act(2)-act(3)+act(4)) ] * F_rcs * L_rcs;
            p(i) = p(i-1) + (tau(1)/I_rocket)*dt;
            q(i) = q(i-1) + (tau(2)/I_rocket)*dt;
            r(i) = r(i-1) + (tau(3)/I_rocket)*dt;
            gas_rcs_total(i) = gas_rcs_total(i-1) + sum(act)*m_dot_rcs*dt;
        end

        v(i) = v(i-1) + accel(i)*dt;
        h(i) = h(i-1) + v(i)*dt;
        phi(i) = phi(i-1) + p(i)*dt;
        theta(i) = theta(i-1) + q(i)*dt;
        psi(i) = psi(i-1) + r(i)*dt;

        gas_main(i) = gas_main(i-1) + is_burning*m_dot_main*dt;
        mass(i) = m_now - (gas_main(i)-gas_main(i-1)) - (gas_rcs_total(i)-gas_rcs_total(i-1));

        if h(i) <= 0
            landed = true;
            idx_land = i;
            h(i)=0; v(i)=0; accel(i)=0;
        end
    else
        h(i)=0; v(i)=0; accel(i)=0; mass(i)=mass(i-1);
    end
end

% --- Dashboard (3x2 layout) ---
t_p = t(1:idx_land);
f = figure('Name','Ground Control Station','Color',[0.1 0.1 0.1],'Position',[50 50 1200 800]);

ax1 = subplot(3,2,1);
plot(t_p, h(1:idx_land), 'c', 'LineWidth', 2);
title(ax1, 'Altitude (m)', 'Color', 'w'); ylabel(ax1, 'Meters', 'Color', 'w');

ax2 = subplot(3,2,2);
plot(t_p, rad2deg(phi(1:idx_land)), 'r', t_p, rad2deg(theta(1:idx_land)), 'g', t_p, rad2deg(psi(1:idx_land)), 'y', 'LineWidth', 1.5);
legend(ax2, {'Roll','Pitch','Yaw'},'TextColor','w','Color','none');
title(ax2, 'Attitude (deg)', 'Color', 'w'); ylabel(ax2, 'Degrees', 'Color', 'w');

ax3 = subplot(3,2,3);
plot(t_p, v(1:idx_land), 'm', 'LineWidth', 2);
title(ax3, 'Vertical Speed (m/s)', 'Color', 'w'); ylabel(ax3, 'm/s', 'Color', 'w');

ax4 = subplot(3,2,4);
hold(ax4, 'on');
for k=1:4
    plot(t_p, rcs_states(k,1:idx_land)*0.6 + k, 'LineWidth', 1.5);
end
set(ax4, 'YTick', 1:4, 'YTickLabel', {'RCS 1','RCS 2','RCS 3','RCS 4'});
title(ax4, 'RCS Valve Timing', 'Color', 'w'); ylabel(ax4, 'Valve #', 'Color', 'w');

ax5 = subplot(3,2,5);
plot(t_p, accel(1:idx_land), 'w', 'LineWidth', 1.5);
yline(ax5, 0, '--r', 'LineWidth', 1.5);
title(ax5, 'Net Acceleration (G)', 'Color', 'w'); ylabel(ax5, 'm/s^2', 'Color', 'w');
xlabel(ax5, 'Time (s)', 'Color', 'w');

ax6 = subplot(3,2,6);
hold(ax6, 'on');
area(ax6, t_p, gas_main(1:idx_land), 'FaceColor', [0.8 0.2 0.2], 'FaceAlpha', 0.5);
area(ax6, t_p, gas_rcs_total(1:idx_land), 'FaceColor', [0.2 0.2 0.8], 'FaceAlpha', 0.5);
legend(ax6, {'Main Engine','RCS'},'TextColor','w','Color','none', 'Location', 'northwest');
title(ax6, 'Propellant Consumption (kg)', 'Color', 'w'); ylabel(ax6, 'kg', 'Color', 'w');
xlabel(ax6, 'Time (s)', 'Color', 'w');

sgtitle('6-DOF HYBRID CONTROL ANALYSIS PANEL', 'Color', 'w', 'FontSize', 16, 'FontWeight', 'bold');

% Dark theme for all axes
axes_list = [ax1, ax2, ax3, ax4, ax5, ax6];
for i = 1:length(axes_list)
    ax = axes_list(i);
    grid(ax, 'on');
    set(ax, 'Color', [0.15 0.15 0.15], 'XColor', 'w', 'YColor', 'w', ...
        'GridColor', 'w', 'GridAlpha', 0.2, 'LineWidth', 1);
end
