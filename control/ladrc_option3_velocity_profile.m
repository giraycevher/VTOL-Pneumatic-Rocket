% LADRC + PWM on/off vertical landing simulation.
% Option 3: continuous velocity profile (safest) - a linear altitude-based
% target speed from -2.0 m/s at 7.77 m to -0.2 m/s at touchdown, tracked
% by LADRC on the Z axis and LQR+LESO on pitch/roll.
clear; clc; close all;
addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'utils'));

% --- Vehicle parameters ---
m0          = 27.674;
m_dot_total = 0.640;
g           = 9.81;

Ixx = 3.24;
Iyy = 3.24;
Izz = 0.45;

F_main_total = 344.32;
n_main       = 4;

F_rcs_per    = 25.27;
d_rcs        = 0.40;

t_valve_open  = 0.005;
t_valve_close = 0.005;
P_reg = 10;

% --- Simulation parameters ---
dt       = 0.001;
dt_ctrl  = 0.010;
t_end    = 12.0;
t        = 0:dt:t_end;
N        = length(t);

z0       = 7.77;
vz0      = -3.5;

theta0       = 2 * pi/180;
phi0         = -1 * pi/180;
omega_theta0 = 0;
omega_phi0   = 0;

% --- PWM parameters ---
pwm_window = 10;

pwm_levels = [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0];

pwm_patterns = [
    0 0 0 0 0 0 0 0 0 0;   % 0%
    1 0 0 0 0 0 0 0 0 0;   % 10%
    1 0 0 0 0 1 0 0 0 0;   % 20%
    1 0 0 1 0 0 1 0 0 0;   % 30%
    1 0 1 0 1 0 1 0 0 0;   % 40%
    1 0 1 0 1 0 1 0 1 0;   % 50%
    1 1 0 1 0 1 1 0 1 0;   % 60%
    1 1 0 1 1 0 1 1 0 1;   % 70%
    1 1 1 0 1 1 1 0 1 1;   % 80%
    1 1 1 1 0 1 1 1 1 1;   % 90%
    1 1 1 1 1 1 1 1 1 1;   % 100%
];

% --- LADRC parameters ---
b0_z    = F_main_total / m0;
wc_z    = 10;
wo_z    = 5 * wc_z;

b0_theta = d_rcs * F_rcs_per / Iyy;
wc_theta = 15;
wo_theta = 5 * wc_theta;

b0_phi   = d_rcs * F_rcs_per / Ixx;
wc_phi   = 15;
wo_phi   = 5 * wc_phi;

% Velocity profile parameters
vz_max  = -2.0;   % fastest descent rate [m/s]
vz_min  = -0.2;   % touchdown target [m/s]
z_start = 7.77;   % profile start altitude [m]

fprintf('=== OPTION 3: CONTINUOUS VELOCITY PROFILE ===\n');
fprintf('Strategy: altitude-dependent linear target speed\n');
fprintf('  %.1fm -> %.1f m/s\n', z_start, vz_max);
fprintf('  0.0m  -> %.1f m/s\n', vz_min);
fprintf('\nFormula: vz_ref = %.1f - (z / %.2f) * %.1f\n\n', ...
    vz_min, z_start, abs(vz_max - vz_min));
fprintf('LADRC parameters:\n');
fprintf('Z axis:     b0=%.2f, wc=%.1f rad/s, wo=%.1f rad/s\n', b0_z, wc_z, wo_z);
fprintf('Pitch axis: b0=%.2f, wc=%.1f rad/s, wo=%.1f rad/s\n', b0_theta, wc_theta, wo_theta);
fprintf('Roll axis:  b0=%.2f, wc=%.1f rad/s, wo=%.1f rad/s\n\n', b0_phi, wc_phi, wo_phi);

% --- LESO gains ---
beta1_z = 3*wo_z;      beta2_z = 3*wo_z^2;      beta3_z = wo_z^3;
beta1_t = 3*wo_theta;  beta2_t = 3*wo_theta^2;  beta3_t = wo_theta^3;
beta1_p = 3*wo_phi;    beta2_p = 3*wo_phi^2;    beta3_p = wo_phi^3;

kp_z = wc_z^2;         kd_z = 2*wc_z;
kp_t = wc_theta^2;     kd_t = 2*wc_theta;
kp_p = wc_phi^2;       kd_p = 2*wc_phi;

% --- LQR gains (pitch/roll, inverted-pendulum model) ---
L_cg = 1.2; % distance from center of mass

A_pitch = [0, 1;
           (m0 * g * L_cg)/Iyy, 0];
B_pitch = [0;
           b0_theta];

A_roll = [0, 1;
          (m0 * g * L_cg)/Ixx, 0];
B_roll = [0;
          b0_phi];

% Q/R weights (tuned via Bryson's rule, then hand-adjusted to reach 0.2 m/s)
Q_lqr = diag([1e3, 10]);
R_lqr = 1;

K_pitch = lqr(A_pitch, B_pitch, Q_lqr, R_lqr);
K_roll  = lqr(A_roll,  B_roll,  Q_lqr, R_lqr);

fprintf('K_pitch = [%.2f, %.2f]\n', K_pitch(1), K_pitch(2));
fprintf('K_roll  = [%.2f, %.2f]\n\n', K_roll(1), K_roll(2));

% --- State vectors ---
z_pos   = zeros(N,1);  z_pos(1) = z0;
vz      = zeros(N,1);  vz(1)    = vz0;
theta   = zeros(N,1);  theta(1) = theta0;
omega_t = zeros(N,1);  omega_t(1) = omega_theta0;
phi     = zeros(N,1);  phi(1)   = phi0;
omega_p = zeros(N,1);  omega_p(1) = omega_phi0;
mass    = zeros(N,1);  mass(1)  = m0;

z1_z = z0;   z2_z = vz0;  z3_z = 0;
z1_t = theta0; z2_t = 0;  z3_t = 0;
z1_p = phi0;   z2_p = 0;  z3_p = 0;

u_z_cont     = zeros(N,1);
u_theta_cont = zeros(N,1);
u_phi_cont   = zeros(N,1);

valve_main = zeros(N,1);
valve_rcs  = zeros(N,4);

vz_ref_log = zeros(N,1);

pwm_counter   = 0;
pwm_pattern_z = zeros(1, pwm_window);
pwm_pattern_t = zeros(1, pwm_window);
pwm_pattern_p = zeros(1, pwm_window);

% --- Disturbance model ---
rng(42);
wind_force        = 5 * sin(2*pi*0.5*t) + 2*randn(1,N);
wind_moment_pitch = 0.3 * sin(2*pi*0.8*t) + 0.1*randn(1,N);
wind_moment_roll  = 0.2 * sin(2*pi*0.6*t) + 0.1*randn(1,N);

% --- Main simulation loop ---
fprintf('Simulation starting...\n');
landed    = false;
ctrl_step = 0;

for k = 1:N-1

    if valve_main(k) > 0
        mass(k+1) = mass(k) - m_dot_total * dt;
    else
        mass(k+1) = mass(k);
    end
    mass(k+1) = max(mass(k+1), m0 - m_dot_total * 3);
    m_k = mass(k);

    if z_pos(k) < 1.5
        k_ground = 1 + 0.15 * exp(-2 * z_pos(k));
    else
        k_ground = 1.0;
    end

    F_thrust_z = valve_main(k) * F_main_total * k_ground * cos(theta(k)) * cos(phi(k));
    F_gravity  = -m_k * g;
    F_drag     = -0.5 * 1.225 * 0.3 * 0.04 * vz(k) * abs(vz(k));
    F_wind_z   = wind_force(k) * 0.1;
    F_total_z  = F_thrust_z + F_gravity + F_drag + F_wind_z;

    M_rcs_pitch   = (valve_rcs(k,1) - valve_rcs(k,2)) * F_rcs_per * d_rcs;
    M_wind_pitch  = wind_moment_pitch(k);
    M_total_pitch = M_rcs_pitch + M_wind_pitch;

    M_rcs_roll   = (valve_rcs(k,3) - valve_rcs(k,4)) * F_rcs_per * d_rcs;
    M_wind_roll  = wind_moment_roll(k);
    M_total_roll = M_rcs_roll + M_wind_roll;

    az = F_total_z / m_k;
    vz(k+1)    = vz(k) + az * dt;
    z_pos(k+1) = z_pos(k) + vz(k) * dt;

    alpha_theta  = M_total_pitch / Iyy;
    omega_t(k+1) = omega_t(k) + alpha_theta * dt;
    theta(k+1)   = theta(k) + omega_t(k) * dt;

    alpha_phi    = M_total_roll / Ixx;
    omega_p(k+1) = omega_p(k) + alpha_phi * dt;
    phi(k+1)     = phi(k) + omega_p(k) * dt;

    if z_pos(k+1) <= 0.02
        z_pos(k+1)   = 0;
        vz(k+1)      = 0;
        omega_t(k+1) = 0;
        omega_p(k+1) = 0;
        if ~landed
            fprintf('TOUCHDOWN! t=%.3f s, speed=%.3f m/s\n', t(k+1), vz(k));
            landed = true;
        end
    end

    if mod(k-1, round(dt_ctrl/dt)) == 0 && ~landed
        ctrl_step = ctrl_step + 1;

        y_z     = z_pos(k) + 0.01*randn;
        y_vz    = vz(k)    + 0.05*randn;
        y_theta = theta(k) + 0.002*randn;
        y_wt    = omega_t(k) + 0.01*randn;
        y_phi   = phi(k)   + 0.002*randn;
        y_wp    = omega_p(k) + 0.01*randn;

        % LESO - Z axis
        e_z      = y_z - z1_z;
        z1_z_dot = z2_z + beta1_z * e_z;
        z2_z_dot = z3_z + beta2_z * e_z + b0_z * u_z_cont(k);
        z3_z_dot = beta3_z * e_z;
        z1_z = z1_z + z1_z_dot * dt_ctrl;
        z2_z = z2_z + z2_z_dot * dt_ctrl;
        z3_z = z3_z + z3_z_dot * dt_ctrl;

        % Option 3: linear altitude-to-target-speed interpolation
        % 7.77 m -> -2.0 m/s, 0 m -> -0.2 m/s
        z_current = max(0, z1_z);
        vz_ref = vz_min + (z_current / z_start) * (vz_max - vz_min);
        vz_ref = max(-2.0, min(-0.18, vz_ref));
        vz_ref_log(k) = vz_ref;

        e_vz = vz_ref - z2_z;
        u0_z = kd_z * e_vz;
        u_z  = (u0_z - z3_z) / b0_z;
        u_z_normalized = (m_k * (g + u_z)) / F_main_total;
        u_z_normalized = max(0, min(1, u_z_normalized));
        u_z_cont(k) = u_z_normalized;

        % LQR + LESO - Pitch axis
        e_t      = y_theta - z1_t;
        z1_t_dot = z2_t + beta1_t * e_t;
        z2_t_dot = z3_t + beta2_t * e_t + b0_theta * u_theta_cont(k);
        z3_t_dot = beta3_t * e_t;

        z1_t = z1_t + z1_t_dot * dt_ctrl;
        z2_t = z2_t + z2_t_dot * dt_ctrl;
        z3_t = z3_t + z3_t_dot * dt_ctrl;

        % State feedback: x = [angle; rate] -> u_lqr = -K*x
        % Uses the LESO-filtered z1/z2 to stay robust to sensor noise.
        u_t_lqr = -K_pitch(1) * z1_t - K_pitch(2) * z2_t;

        % LESO's z3 term rejects wind and unmodeled dynamics
        u_t = u_t_lqr - (z3_t / b0_theta);

        u_t  = max(-1, min(1, u_t));
        u_theta_cont(k) = u_t;

        % LQR + LESO - Roll axis
        e_p      = y_phi - z1_p;
        z1_p_dot = z2_p + beta1_p * e_p;
        z2_p_dot = z3_p + beta2_p * e_p + b0_phi * u_phi_cont(k);
        z3_p_dot = beta3_p * e_p;

        z1_p = z1_p + z1_p_dot * dt_ctrl;
        z2_p = z2_p + z2_p_dot * dt_ctrl;
        z3_p = z3_p + z3_p_dot * dt_ctrl;

        u_p_lqr = -K_roll(1) * z1_p - K_roll(2) * z2_p;
        u_p = u_p_lqr - (z3_p / b0_phi);

        u_p  = max(-1, min(1, u_p));
        u_phi_cont(k) = u_p;

        % PWM modulation
        pwm_counter = pwm_counter + 1;
        pwm_pos = mod(pwm_counter - 1, pwm_window) + 1;

        if pwm_pos == 1
            pwm_pattern_z = pwm_quantize(u_z_normalized, pwm_patterns, pwm_levels);
            u_t_abs = abs(u_theta_cont(k));
            pwm_pattern_t = pwm_quantize(u_t_abs, pwm_patterns, pwm_levels);
            u_p_abs = abs(u_phi_cont(k));
            pwm_pattern_p = pwm_quantize(u_p_abs, pwm_patterns, pwm_levels);
        end

        valve_main(k+1) = pwm_pattern_z(pwm_pos);

        if u_theta_cont(k) >= 0
            valve_rcs(k+1, 1) = pwm_pattern_t(pwm_pos);
            valve_rcs(k+1, 2) = 0;
        else
            valve_rcs(k+1, 1) = 0;
            valve_rcs(k+1, 2) = pwm_pattern_t(pwm_pos);
        end

        if u_phi_cont(k) >= 0
            valve_rcs(k+1, 3) = pwm_pattern_p(pwm_pos);
            valve_rcs(k+1, 4) = 0;
        else
            valve_rcs(k+1, 3) = 0;
            valve_rcs(k+1, 4) = pwm_pattern_p(pwm_pos);
        end

    else
        if k+1 <= N
            valve_main(k+1)   = valve_main(k);
            valve_rcs(k+1,:)  = valve_rcs(k,:);
            u_z_cont(k+1)     = u_z_cont(k);
            u_theta_cont(k+1) = u_theta_cont(k);
            u_phi_cont(k+1)   = u_phi_cont(k);
        end
    end
end

% --- Result summary ---
land_idx = find(z_pos <= 0.02 & [false; diff(z_pos) < 0], 1, 'first');
if isempty(land_idx), land_idx = N; end
t_land = t(land_idx);
v_land = vz(max(1, land_idx-1));

fprintf('\n=== RESULTS - OPTION 3 (VELOCITY PROFILE) ===\n');
fprintf('Landing time:      %.3f s\n', t_land);
fprintf('Landing speed:     %.3f m/s\n', abs(v_land));
fprintf('Max pitch:         %.2f deg\n', max(abs(theta(1:land_idx)))*180/pi);
fprintf('Max roll:          %.2f deg\n', max(abs(phi(1:land_idx)))*180/pi);
fprintf('Final pitch:       %.2f deg\n', abs(theta(land_idx))*180/pi);
fprintf('Final roll:        %.2f deg\n', abs(phi(land_idx))*180/pi);
fprintf('Propellant used:   %.3f kg\n', m0 - mass(land_idx));
fprintf('Propellant left:   %.3f kg\n', mass(land_idx));

if abs(v_land) <= 0.2
    fprintf('\n>>> PASS: landing speed %.3f m/s meets the 0.2 m/s limit <<<\n', abs(v_land));
elseif abs(v_land) <= 2.0
    fprintf('\n>>> PARTIAL: landing speed %.3f m/s meets the old 2.0 m/s limit but not 0.2 m/s <<<\n', abs(v_land));
else
    fprintf('\n>>> FAIL: landing speed %.3f m/s exceeds both limits <<<\n', abs(v_land));
end

% --- Plots ---
figure('Name','Flight State','Color','w');

subplot(3,1,1);
plot(t, z_pos, 'b-', 'LineWidth', 1.5); hold on;
yline(0, 'r--', 'Ground', 'LineWidth', 1);
xlabel('Time [s]'); ylabel('Altitude [m]');
title('Altitude'); grid on;
xlim([0 t_land+0.5]);

subplot(3,1,2);
plot(t, vz, 'b-', 'LineWidth', 1.5); hold on;
plot(t, vz_ref_log, 'g--', 'LineWidth', 1.5);
yline(-0.2, 'r--', 'New Limit', 'LineWidth', 1.2);
yline(-2.0, 'm--', 'Old Limit', 'LineWidth', 1);
yline(0, 'k-');
xlabel('Time [s]'); ylabel('Speed [m/s]');
title('Vertical Speed vs Target Profile');
legend('Actual Speed','Target Profile','Location','best');
grid on;
xlim([0 t_land+0.5]);

subplot(3,1,3);
plot(t, mass, 'r-', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Mass [kg]');
title('Mass');
grid on;
xlim([0 t_land+0.5]);

figure('Name','Attitude and Control','Color','w');

subplot(3,1,1);
plot(t, theta*180/pi, 'b-', 'LineWidth', 1.5); hold on;
yline(0,'k--');
xlabel('Time [s]'); ylabel('Pitch [deg]');
title('Pitch Angle');
grid on;
xlim([0 t_land+0.5]);

subplot(3,1,2);
plot(t, phi*180/pi, 'r-', 'LineWidth', 1.5); hold on;
yline(0,'k--');
xlabel('Time [s]'); ylabel('Roll [deg]');
title('Roll Angle');
grid on;
xlim([0 t_land+0.5]);

subplot(3,1,3);
plot(t, u_z_cont, 'b-', 'LineWidth', 1.2); hold on;
plot(t, u_theta_cont, 'r-', 'LineWidth', 1.2);
plot(t, u_phi_cont, 'g-', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('Control (norm)');
title('LADRC Control Outputs');
legend('Z (main)','Pitch','Roll','Location','best');
grid on;
xlim([0 t_land+0.5]);

figure('Name','PWM and Valves','Color','w');

subplot(3,1,1);
stairs(t, valve_main, 'b-', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('Valve');
title('Main Engine PWM');
grid on;
ylim([-0.1 1.1]);
xlim([0 min(t_land,3)]);

subplot(3,1,2);
stairs(t, valve_rcs(:,1), 'b-', 'LineWidth', 1.2); hold on;
stairs(t, -valve_rcs(:,2), 'r-', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('Valve');
title('Pitch RCS');
legend('+X','-X','Location','best');
grid on;
xlim([0 min(t_land,3)]);

subplot(3,1,3);
stairs(t, valve_rcs(:,3), 'b-', 'LineWidth', 1.2); hold on;
stairs(t, -valve_rcs(:,4), 'r-', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('Valve');
title('Roll RCS');
legend('+Y','-Y','Location','best');
grid on;
xlim([0 min(t_land,3)]);
