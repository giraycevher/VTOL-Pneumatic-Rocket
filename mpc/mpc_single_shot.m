% Single-shot nonlinear MPC trajectory/control computation for a
% free-fall-to-landing maneuver.
clear; clc; close all;

% 1. Vehicle parameters
m0 = 27.674;              % initial mass (kg)
F_max_main = 344.32;      % max main engine thrust (N)
F_max_rcs  = 25.27;       % max RCS thruster force (N)

% 2. nlmpc object
nx = 7; % states: px, vx, py, vy, theta, omega, mass
nu = 2; % inputs: F_main, F_rcs
nlobj = nlmpc(nx, nx, nu);

nlobj.Ts = 0.1;
nlobj.PredictionHorizon = 100;
nlobj.ControlHorizon = 70;

nlobj.Model.StateFcn = @rocket_rcs_dynamics;

% 3. Constraints
nlobj.MV(1).Min = 0;              nlobj.MV(1).Max = F_max_main; % main engine cannot pull
nlobj.MV(2).Min = -F_max_rcs;     nlobj.MV(2).Max = F_max_rcs;
nlobj.States(3).Min = 0;          % altitude cannot go below ground
nlobj.States(7).Min = 20.0;       % mass floor (dry mass)

% 4. Cost function
% [px, vx, py, vy, theta, omega, mass]
nlobj.Weights.OutputVariables = [5 2 20 10 30 5 0];
nlobj.Weights.ManipulatedVariablesRate = [0.1 0.1];

% 5. Initial conditions
% Vehicle at x=0, y=10 m. All rates and angles zero.
x0 = [0; 0; 10; 0; 0; 0; m0];
u0 = [0; 0];

% Target
yref = [0 0 0.1 0 0 0 m0];

% 6. Solve
fprintf('Computing 10 m free-fall landing trajectory...\n');
[u_opt, options, info] = nlmpcmove(nlobj, x0, u0, yref);
fprintf('Done.\n');

x_opt = info.Xopt;
u_opt = info.MVopt;
t = 0:0.1:(size(x_opt,1)-1)*0.1;

% --- Plots ---
figure('Name','Suicide Burn MPC','Color','w','Position',[100 100 1200 800]);

subplot(2,2,1);
plot(x_opt(:,1), x_opt(:,3), 'b-', 'LineWidth', 2); hold on;
plot(x_opt(1,1), x_opt(1,3), 'go', 'MarkerSize', 8, 'LineWidth', 2);
plot(0, 0, 'rx', 'MarkerSize', 12, 'LineWidth', 2);
title('2D Landing Trajectory');
xlabel('X Position (m)'); ylabel('Altitude (m)');
grid on;

subplot(2,2,2);
yyaxis left
plot(t, x_opt(:,3), 'b-', 'LineWidth', 2); ylabel('Altitude (m)');
yyaxis right
plot(t, x_opt(:,4), 'r-', 'LineWidth', 2); ylabel('Vertical Speed (m/s)');
title('Altitude and Speed Profile'); xlabel('Time (s)');
grid on;

subplot(2,2,3);
stairs(t, u_opt(:,1), 'b-', 'LineWidth', 2); hold on;
yline(F_max_main, 'r--', 'Max Thrust', 'LineWidth', 1.5);
title('Main Engine Command (MPC Output)');
xlabel('Time (s)'); ylabel('Force (N)');
ylim([-10 400]); grid on;

subplot(2,2,4);
plot(t, x_opt(:,7), 'm-', 'LineWidth', 2);
title('Vehicle Mass');
xlabel('Time (s)'); ylabel('Mass (kg)');
grid on;
