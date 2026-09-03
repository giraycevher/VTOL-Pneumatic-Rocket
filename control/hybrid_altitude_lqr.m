% Altitude-axis hybrid landing controller: an analytic suicide-burn
% trigger hands off to an LQR hover/terminal-descent law with valve
% hysteresis and an anti-bounce lockout.
clear; clc; close all;

% --- Physical parameters ---
m0 = 27.674;
F_thrust = 600;
g = 9.81;
m_dot = 0.734;
H_start = 10;

% Valve parameters
t_on_delay = 0.200;
t_off_delay = 0.200;
min_stay_time = 0.350;   % minimum on-time away from the ground

% LQR
A = [0 1; 0 0]; B = [0; 1/m0];
Q = eig(1300,5300);
R = 0.3;
K_lqr = lqr(A, B, Q, R);

% Simulation setup
dt = 0.001;
t = 0:dt:10;

h = zeros(size(t)); v = zeros(size(t)); m = zeros(size(t));
valve_state = zeros(size(t));
h(1) = H_start;
v(1) = 0;
m(1) = m0;

suicide_burn_started = false;
last_switch_time = -min_stay_time;

for i = 2:length(t)

    h_now = h(i-1); v_now = v(i-1); m_now = m(i-1);
    mg = m_now * g;
    a_max = (F_thrust / m_now) - g;

    % Dynamic target speed: scales down as the ground gets closer
    v_target = -0.8 * h_now;
    if v_target < -2.0, v_target = -2.0; end
    if v_target > -0.10, v_target = -0.10; end   % minimum touchdown glide speed

    % Control logic
    prev_valve = valve_state(i-1);
    valve_cmd = prev_valve;

    % Analytic stopping distance for the suicide burn (with a margin)
    h_stop = (v_now^2 / (2 * a_max)) * 1.05 + abs(v_now * t_on_delay) + 0.2;

    if ~suicide_burn_started
        if h_now <= h_stop
            suicide_burn_started = true;
            valve_cmd = 1;
            last_switch_time = t(i);
        else
            valve_cmd = 0;
        end
    else
        % LQR on the terminal descent
        F_lqr = -K_lqr * [h_now - 0; v_now - v_target] + mg;

        % Allow faster valve cycling within 1 m of the ground
        current_min_stay = min_stay_time;
        if h_now < 1.0
            current_min_stay = 0.020;
        end

        if (t(i) - last_switch_time) >= current_min_stay
            if prev_valve == 0 && F_lqr > mg * 1.02
                valve_cmd = 1;
                last_switch_time = t(i);
            elseif prev_valve == 1 && F_lqr < mg * 0.98
                valve_cmd = 0;
                last_switch_time = t(i);
            end
        end

        % Anti-bounce lockout: cut the valve if the vehicle starts climbing
        if v_now > -0.05 && h_now > 0.01
            if valve_cmd == 1
                valve_cmd = 0;
                last_switch_time = t(i);
            end
        end
    end

    % Physics
    valve_state(i) = valve_cmd;
    a = (valve_cmd * F_thrust / m_now) - g;
    m(i) = m_now - (valve_cmd * m_dot * dt);
    v(i) = v_now + a * dt;
    h(i) = h_now + v(i) * dt;

    if mod(i, 100) == 0
        fprintf('t: %5.2f s | h: %6.3f m | v: %6.3f m/s | valve: %d\n', t(i), h(i), v(i), valve_cmd);
    end

    if h(i) <= 0
        h(i) = 0;

        total_valve_open_time = sum(valve_state(1:i)) * dt;

        fprintf('\n--- TOUCHDOWN ---\n');
        fprintf('Total flight time     : %.3f s\n', t(i));
        fprintf('Total valve open time : %.3f s\n', total_valve_open_time);
        fprintf('Landing speed         : %.3f m/s\n', abs(v(i)));

        if abs(v(i)) <= 0.2
            disp('RESULT: SOFT LANDING (speed <= 0.2 m/s)');
        else
            disp('RESULT: FAILED (hard landing, limit exceeded)');
        end

        v(i) = 0;
        break;
    end
end

t = t(1:i); h = h(1:i); v = v(1:i); valve_state = valve_state(1:i);

% --- Plots ---
figure('Color','w','Name','Stabilized Landing');
subplot(3,1,1); plot(t,h,'b','LineWidth',2); grid on; ylabel('Altitude (m)');
title(sprintf('Rocket Landing (touchdown speed: %.3f m/s | valve open: %.3fs)', abs(v(end)), sum(valve_state)*dt));
subplot(3,1,2); plot(t,v,'r','LineWidth',2); grid on; ylabel('Speed (m/s)');
yline(-0.2, 'c--', 'Limit (-0.2 m/s)', 'LineWidth', 1.5);
subplot(3,1,3); plot(t,valve_state,'g','LineWidth',1.5);
grid on; ylim([-0.2 1.2]); ylabel('Valve (0-1)'); xlabel('Time (s)');
