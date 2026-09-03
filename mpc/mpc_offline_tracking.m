% Two-stage MPC landing: an offline nonlinear MPC solve produces a
% reference trajectory once (using a more realistic dynamics model with
% ground effect and aerodynamic drag), then a lightweight PD law tracks
% that reference in closed loop, avoiding an online NLP solve.
clear; clc; close all;

% --- Stage 1: offline reference trajectory via MPC ---
fprintf('Stage 1: computing offline MPC landing trajectory...\n');

m0 = 27.674;
F_max_main = 344.32;
F_max_rcs  = 25.27;

nx = 7; % px, vx, py, vy, theta, omega, mass
nu = 2; % F_main, F_rcs
nlobj = nlmpc(nx, nx, nu);
nlobj.Ts = 0.1;
nlobj.PredictionHorizon = 100;
nlobj.ControlHorizon = 70;
nlobj.Model.StateFcn = @rocket_dynamics_ground_effect;

nlobj.MV(1).Min = 0;              nlobj.MV(1).Max = F_max_main;
nlobj.MV(2).Min = -F_max_rcs;     nlobj.MV(2).Max = F_max_rcs;
nlobj.States(3).Min = 0;          % altitude floor
nlobj.States(7).Min = 20.0;       % dry mass floor

% Weight vy heavily to keep the landing speed within spec
nlobj.Weights.OutputVariables = [5, 2, 20, 50, 30, 5, 0];
nlobj.Weights.ManipulatedVariablesRate = [0.1, 0.1];

x0 = [0; 0; 10; 0; 0; 0; m0]; % 10 m free fall
u0 = [0; 0];
yref = [0, 0, 0, -0.15, 0, 0, m0]; % target landing speed -0.15 m/s

[~, options, info] = nlmpcmove(nlobj, x0, u0, yref);
X_ref = info.Xopt;
U_ref = info.MVopt;
time_grid = 0:0.1:(size(X_ref,1)-1)*0.1;

fprintf('Reference trajectory computed. Running closed-loop tracking...\n');

% --- Stage 2: closed-loop tracking simulation ---
n_steps = length(time_grid) - 1;
X_actual = zeros(n_steps+1, nx);
X_actual(1,:) = x0';

U_applied = zeros(n_steps, nu);

% PD tracking gains
K_z = [15, 8];   % [altitude error, vertical speed error]
K_rcs = [20, 5]; % [angle error, angular rate error]

for k = 1:n_steps
    state_now = X_actual(k, :);
    ref_state = X_ref(k, :);
    ref_thrust = U_ref(k, :);

    err_z = ref_state(3) - state_now(3);
    err_vz = ref_state(4) - state_now(4);
    err_theta = ref_state(5) - state_now(5);
    err_omega = ref_state(6) - state_now(6);

    correction_main = K_z(1)*err_z + K_z(2)*err_vz;
    correction_rcs = K_rcs(1)*err_theta + K_rcs(2)*err_omega;

    cmd_main = ref_thrust(1) + correction_main;
    cmd_rcs = ref_thrust(2) + correction_rcs;

    cmd_main = max(0, min(F_max_main, cmd_main));
    cmd_rcs = max(-F_max_rcs, min(F_max_rcs, cmd_rcs));

    U_applied(k, :) = [cmd_main, cmd_rcs];

    dt = 0.1;
    deriv = rocket_dynamics_ground_effect(state_now', U_applied(k, :)');
    X_actual(k+1, :) = state_now + (deriv' * dt);

    if X_actual(k+1, 3) <= 0.05
        X_actual(k+1, 3) = 0;
        fprintf('\n>>> TOUCHDOWN <<<\n');
        fprintf('Landing speed: %.3f m/s\n', X_actual(k+1, 4));
        if abs(X_actual(k+1, 4)) <= 0.20
            fprintf('Specification met (<= 0.2 m/s)\n');
        else
            fprintf('WARNING: exceeds 0.2 m/s limit\n');
        end
        break;
    end
end

% --- Plots ---
figure('Name','Closed-Loop Flight Result','Color','w','Position',[100 100 1200 800]);

subplot(2,2,1);
plot(time_grid(1:k), X_actual(1:k, 3), 'b-', 'LineWidth', 2); hold on;
plot(time_grid(1:k), X_ref(1:k, 3), 'r--', 'LineWidth', 1.5);
title('Altitude Tracking'); xlabel('Time (s)'); ylabel('Altitude (m)');
legend('Actual', 'MPC Reference'); grid on;

subplot(2,2,2);
plot(time_grid(1:k), X_actual(1:k, 4), 'b-', 'LineWidth', 2); hold on;
plot(time_grid(1:k), X_ref(1:k, 4), 'r--', 'LineWidth', 1.5);
yline(-0.2, 'g--', 'Landing Speed Limit (-0.2 m/s)', 'LineWidth', 2);
title('Vertical Speed Tracking'); xlabel('Time (s)'); ylabel('Speed (m/s)');
legend('Actual', 'Reference', 'Limit'); grid on;

subplot(2,2,3);
stairs(time_grid(1:k), U_applied(1:k, 1), 'b-', 'LineWidth', 2); hold on;
yline(F_max_main, 'r--', 'Max Thrust', 'LineWidth', 1.5);
title('Main Engine Command'); xlabel('Time (s)'); ylabel('Force (N)');
grid on;

subplot(2,2,4);
plot(time_grid(1:k), X_actual(1:k, 7), 'm-', 'LineWidth', 2);
title('Remaining Mass'); xlabel('Time (s)'); ylabel('Mass (kg)');
grid on;

function dxdt = rocket_dynamics_ground_effect(x, u)
% Planar rocket dynamics with a near-ground thrust boost (ground effect)
% and quadratic aerodynamic drag, used as the MPC prediction model and
% the "real" plant in the tracking loop.
    g           = 9.81;
    m_dot_total = 0.640;
    F_max_main  = 344.32;
    I_pitch     = 3.24;
    d_rcs       = 0.40;

    rho = 1.225;
    Cd = 0.7;
    A = 0.03;

    c = m_dot_total / F_max_main;

    h     = x(3);
    v_y   = x(4);
    theta = x(5);
    m     = x(7);

    F_m_cmd = u(1);
    F_r     = u(2);

    % Ground effect: thrust rises within the last 0.5 m (air-cushion effect)
    if h < 0.5 && h > 0
        k_ground = 1 + (0.05 / (h + 0.01));
    else
        k_ground = 1.0;
    end

    F_m_actual = F_m_cmd * k_ground;

    D = 0.5 * rho * v_y^2 * Cd * A * sign(v_y);

    dxdt = zeros(7,1);
    dxdt(1) = x(2);
    dxdt(2) = (F_m_actual * sin(theta) + F_r * cos(theta)) / m;
    dxdt(3) = x(4);
    dxdt(4) = (F_m_actual * cos(theta) - F_r * sin(theta) - D) / m - g;
    dxdt(5) = x(6);
    dxdt(6) = (F_r * d_rcs) / I_pitch;

    % Mass loss is driven only by the commanded valve opening; the extra
    % ground-effect thrust does not cost additional propellant.
    dxdt(7) = -c * F_m_cmd - (c * 0.2) * abs(F_r);
end
