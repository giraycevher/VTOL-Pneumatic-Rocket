function dxdt = rocket_rcs_dynamics(x, u)
% Simplified 7-state planar rocket dynamics used as the nlmpc prediction
% model. State: [px, vx, py, vy, theta, omega, mass]. Input: [F_main, F_rcs].
    g           = 9.81;
    m_dot_total = 0.640;
    F_max_main  = 344.32;
    I_pitch     = 3.24;
    d_rcs       = 0.40;

    c = m_dot_total / F_max_main;

    theta = x(5);
    m     = x(7);
    F_m   = u(1);
    F_r   = u(2);

    dxdt = zeros(7,1);
    dxdt(1) = x(2);
    dxdt(2) = (F_m * sin(theta) + F_r * cos(theta)) / m;
    dxdt(3) = x(4);
    dxdt(4) = (F_m * cos(theta) - F_r * sin(theta)) / m - g;
    dxdt(5) = x(6);
    dxdt(6) = (F_r * d_rcs) / I_pitch;
    dxdt(7) = -c * F_m;
end
