% Flagship landing controller (v6): hoverslam + delay-predictive trigger.
% Adds a 350 ms valve-opening delay to the plant model and fires the
% suicide burn early enough that the post-delay state lands exactly on
% the optimal braking curve, followed by a low-altitude PD hover phase.
% Reference result: ~0.15 m/s landing speed, ~59% propellant remaining
% (vs. ~5.7% in earlier versions without the delay-predictive trigger).
clear; clc; close all;

fprintf('=================================================\n');
fprintf('  LANDING CONTROLLER v6 - DELAY-PREDICTIVE SIM\n');
fprintf('  Valve delay: 350ms included\n');
fprintf('=================================================\n\n');

%% 1. System parameters
m_dry    = 19.674;
m_gas_0  = 2.83;
m_0      = m_dry + m_gas_0;      % 22.504 kg
g        = 9.81;
F_max    = 600.0;                % [N]
Ve       = 344.32 / 0.640;       % [m/s] exhaust velocity = 537.9 m/s
mdot_max = F_max / Ve;           % [kg/s] = 1.115 kg/s
h_0      = 9.0;                  % [m] release altitude

% Derived
TWR        = F_max / (m_0*g);
a_net_max  = F_max/m_0 - g;      % 16.85 m/s2
hover_duty = m_0*g / F_max;
mdot_hover = hover_duty * mdot_max;
t_hover_max= m_gas_0 / mdot_hover;

% Tank
V_tank=9e-3; P_tank_0=300e5; R_gas=287; T_tank=303; Z_comp=1.109;

%% 2. Control parameters
t_delay  = 0.350;               % [s] valve opening delay (300-500ms)

% Trigger (delay-predictive): command point h=6.471m, v=7.044 m/s (down)
% 350ms later: h=3.41m, v=10.48 m/s -> braking begins
h_cmd    = 6.471;               % [m] command altitude
v_cmd    = 7.044;               % [m/s] speed at command time

h_pd     = 0.9;                 % [m] altitude for PD hand-off
v_ref_pd = -0.15;                % [m/s] target landing speed
kp_pd    = 50.0;                 % PD gain

dt    = 0.00005;                % [s] 20 kHz
t_max = 10.0;

%% 3. Configuration summary
fprintf('--- SYSTEM ---\n');
fprintf('  Total mass      : %.3f kg\n', m_0);
fprintf('  Max thrust      : %.1f N\n', F_max);
fprintf('  mdot_max        : %.4f kg/s\n', mdot_max);
fprintf('  TWR             : %.3f\n', TWR);
fprintf('  a_net_max       : %.3f m/s2\n', a_net_max);
fprintf('  Hover duty      : %.4f (%.1f%%)\n', hover_duty, hover_duty*100);
fprintf('--- DELAY-PREDICTIVE TRIGGER ---\n');
fprintf('  Valve delay     : %.0f ms\n', t_delay*1000);
fprintf('  Command h       : %.3f m\n', h_cmd);
fprintf('  Command v       : %.3f m/s\n', v_cmd);
fprintf('  After delay     : h~3.41 m, v~10.48 m/s\n');
fprintf('  PD hand-off h   : %.1f m\n', h_pd);
fprintf('\n');

%% 4. Initial conditions
% Vehicle at h_cmd descending at v_cmd - command has been given, delay starts
h = h_cmd;  v = -v_cmd;  m = m_0;  m_gas = m_gas_0;
t = 0.0;    phase = 1;  step = 0;

% Extended state observer
z1_eso=h_cmd; z2_eso=-v_cmd; z3_eso=0.0;
l1=80; l2=1600;

log_stride=10;
N_est=ceil(t_max/dt/log_stride)+1000;
t_log  =zeros(1,N_est); h_log  =zeros(1,N_est);
v_log  =zeros(1,N_est); a_log  =zeros(1,N_est);
F_log  =zeros(1,N_est); thr_log=zeros(1,N_est);
mg_log =zeros(1,N_est); phase_log=zeros(1,N_est);
P_log  =zeros(1,N_est); m_log  =zeros(1,N_est);
z2_log =zeros(1,N_est); z3_log =zeros(1,N_est);
vref_log=zeros(1,N_est);
idx=1;

v_land=0; t_land=0; m_gas_final=0;
throttle=0.0; v_ref_cur=-v_cmd;

%% 5. Simulation loop
% Phase 1: delay window (0 - 350ms) - still in free fall
% Phase 2: full thrust (350ms - h<0.9m)
% Phase 3: PD hover (h < 0.9m - touchdown)
fprintf('Simulation starting...\n');

while t < t_max

    P_tank = max(0, Z_comp * m_gas * R_gas * T_tank / V_tank);

    h_meas = h + 0.01*randn();
    h_meas = max(0, h_meas);

    e_eso = h_meas - z1_eso;
    u_eso = F_log(max(1,idx-1)) / max(m,0.1);
    z1_eso = z1_eso + (z2_eso + 80*e_eso)*dt;
    z2_eso = z2_eso + (z3_eso + u_eso - g + 1600*e_eso)*dt;
    z3_eso = z3_eso + (-0.01*z3_eso)*dt;

    if m_gas <= 0.001
        throttle  = 0.0;
        v_ref_cur = 0.0;
        phase = 4;  % out of propellant

    elseif phase == 1
        % Delay window - valve not yet open
        throttle  = 0.0;
        v_ref_cur = v_ref_pd;
        if t >= t_delay
            phase = 2;
        end

    elseif phase == 2
        % Full thrust - rapid deceleration
        if h > h_pd
            throttle  = 1.0;
            v_ref_cur = v_ref_pd;
        else
            phase = 3;
            throttle  = hover_duty;
            v_ref_cur = v_ref_pd;
        end

    elseif phase == 3
        % PD hover - precision landing
        phase     = 3;
        v_ref_cur = v_ref_pd;
        err_v     = v_ref_pd - v;
        throttle  = hover_duty + kp_pd * err_v * m / F_max;
        throttle  = max(0.0, min(1.0, throttle));
    end

    F_thrust = throttle * F_max;
    mdot_c = throttle * mdot_max;
    a      = F_thrust/m - g;

    if mod(step, log_stride)==0 && idx<=N_est
        t_log(idx)  =t;    h_log(idx)  =h;    v_log(idx)  =v;
        a_log(idx)  =a;    F_log(idx)  =F_thrust; thr_log(idx)=throttle;
        mg_log(idx) =m_gas; phase_log(idx)=phase;  P_log(idx)  =P_tank/1e5;
        m_log(idx)  =m;    z2_log(idx) =z2_eso; z3_log(idx)=z3_eso;
        vref_log(idx)=v_ref_cur;
        idx=idx+1;
    end

    h=h+v*dt; v=v+a*dt;
    m_gas=max(0.0, m_gas-mdot_c*dt);
    m=m_dry+m_gas;
    t=t+dt; step=step+1;

    if h<=0.010 && t>0.05
        v_land=v; t_land=t; m_gas_final=m_gas; break;
    end
    if h>h_0+5; fprintf('WARNING: altitude out of range!\n'); break; end
end

N=idx-1;
t_log  =t_log(1:N);   h_log  =h_log(1:N);   v_log  =v_log(1:N);
a_log  =a_log(1:N);   F_log  =F_log(1:N);   thr_log=thr_log(1:N);
mg_log =mg_log(1:N);  phase_log=phase_log(1:N); P_log  =P_log(1:N);
m_log  =m_log(1:N);   z2_log =z2_log(1:N);  z3_log =z3_log(1:N);
vref_log=vref_log(1:N);

%% 6. Results
fprintf('\n=================================================\n');
fprintf('  SIMULATION RESULTS\n');
fprintf('=================================================\n');
fprintf('  Landing time     : %.3f s\n', t_land);
fprintf('  LANDING SPEED    : %.4f m/s\n', abs(v_land));
fprintf('  Propellant left  : %.3f kg (%.1f%%)\n', m_gas_final, m_gas_final/m_gas_0*100);
fprintf('  Propellant used  : %.3f kg\n', m_gas_0-m_gas_final);
fprintf('  Valve delay      : %.0f ms (modeled)\n', t_delay*1000);
fprintf('  Command h        : %.3f m\n', h_cmd);
if abs(v_land)<=0.2
    fprintf('\n  >>> SPEC MET: %.4f m/s <<<\n\n', abs(v_land));
    result_str='SPEC MET'; result_color=[0 0.55 0];
else
    fprintf('\n  >>> SPEC NOT MET: %.4f m/s <<<\n\n', abs(v_land));
    result_str='SPEC NOT MET'; result_color=[0.8 0 0];
end

%% Colors per phase
c_delay = [0.95 0.60 0.10];  % orange - phase 1, delay
c_p2    = [0.85 0.15 0.10];  % red    - phase 2, full thrust
c_p3    = [0.10 0.65 0.25];  % green  - phase 3, PD hover
c_p4    = [0.50 0.50 0.50];  % gray   - out of propellant

idx_p1 = phase_log==1;
idx_p2 = phase_log==2;
idx_p3 = phase_log==3;
idx_p4 = phase_log==4;

mdot_log = thr_log * mdot_max;

%% Figure 1: main 3x3 panel
fig1=figure('Name','Landing Controller v6 - Main Panel',...
    'Position',[30 30 1600 1050],'Color','w');

ax11=subplot(3,3,1); hold on; grid on; box on;
if any(idx_p1); plot(t_log(idx_p1),h_log(idx_p1),'.','Color',c_delay,'MarkerSize',2); end
if any(idx_p2); plot(t_log(idx_p2),h_log(idx_p2),'.','Color',c_p2,'MarkerSize',2); end
if any(idx_p3); plot(t_log(idx_p3),h_log(idx_p3),'.','Color',c_p3,'MarkerSize',2); end
t_delay_end = t_delay;
fill([0 t_delay_end t_delay_end 0],[0 0 h_0+1 h_0+1],...
    c_delay,'FaceAlpha',0.12,'EdgeColor','none');
xline(t_delay,'--','Valve Opened','Color',c_p2,'LineWidth',2,'LabelHorizontalAlignment','left');
yline(h_pd,'--','PD Hand-off','Color',[0.1 0.4 0.9],'LineWidth',1.5,'LabelHorizontalAlignment','left');
plot(0,h_cmd,'r^','MarkerSize',13,'MarkerFaceColor','r');
plot(t_land,0,'k*','MarkerSize',13,'LineWidth',2);
xlabel('Time (s)'); ylabel('Altitude (m)');
title('Altitude vs Time','FontWeight','bold');
legend({'Phase1 Delay','Phase2 Full Thrust','Phase3 PD Hover','Delay Window','Valve Opened','PD Hand-off','Command','Touchdown'},...
    'Location','northeast','FontSize',6);
xlim([0 t_land*1.12]); ylim([-0.3 h_cmd*1.15]);
set(ax11,'FontSize',9);

ax12=subplot(3,3,2); hold on; grid on; box on;
fill([0 t_delay_end t_delay_end 0],[-15 -15 5 5],...
    c_delay,'FaceAlpha',0.12,'EdgeColor','none');
if any(idx_p1); plot(t_log(idx_p1),v_log(idx_p1),'.','Color',c_delay,'MarkerSize',2,'DisplayName','Phase1 Delay'); end
if any(idx_p2); plot(t_log(idx_p2),v_log(idx_p2),'.','Color',c_p2,'MarkerSize',2,'DisplayName','Phase2 Full Thrust'); end
if any(idx_p3); plot(t_log(idx_p3),v_log(idx_p3),'.','Color',c_p3,'MarkerSize',2,'DisplayName','Phase3 PD'); end
plot(t_log,vref_log,'k--','LineWidth',1.5,'DisplayName','v_{ref}');
plot(t_log,z2_log,'m-','LineWidth',0.8,'DisplayName','ESO z2');
yline(-0.2,'g-','Spec 0.2 m/s','LineWidth',2.5,'LabelHorizontalAlignment','right');
yline(0,'k:','LineWidth',1);
xline(t_delay,'--','Color',c_p2,'LineWidth',2);
plot(t_land,v_land,'k*','MarkerSize',13,'LineWidth',2);
xlabel('Time (s)'); ylabel('Vertical Speed (m/s)');
title('Speed vs Time','FontWeight','bold');
legend({'Delay Window','Phase1','Phase2','Phase3','v_{ref}','ESO','Spec'},...
    'Location','southeast','FontSize',7);
xlim([0 t_land*1.12]);
set(ax12,'FontSize',9);

ax13=subplot(3,3,3); hold on; grid on; box on;
fill([0 t_delay_end t_delay_end 0],[-12 -12 20 20],...
    c_delay,'FaceAlpha',0.12,'EdgeColor','none');
plot(t_log,a_log,'b-','LineWidth',1.2);
yline(-g,'r--','-g (free fall)','LineWidth',1.5,'LabelHorizontalAlignment','right');
yline(0,'k:','LineWidth',1);
yline(a_net_max,'g--',sprintf('a_{max}=%.1f m/s2',a_net_max),'LineWidth',1.2,'LabelHorizontalAlignment','right');
xline(t_delay,'--','Color',c_p2,'LineWidth',2);
xlabel('Time (s)'); ylabel('Acceleration (m/s2)');
title('Vertical Acceleration','FontWeight','bold');
xlim([0 t_land*1.12]);
set(ax13,'FontSize',9);

ax21=subplot(3,3,4); hold on; grid on; box on;
fill([0 t_delay_end t_delay_end 0],[0 0 F_max*1.2 F_max*1.2],...
    c_delay,'FaceAlpha',0.12,'EdgeColor','none');
area(t_log,F_log,'FaceColor',[1 0.35 0.1],'FaceAlpha',0.4,'EdgeColor','none');
plot(t_log,F_log,'r-','LineWidth',1.8);
yline(F_max,'r--',sprintf('F_{max}=%.0f N',F_max),'LineWidth',1.5,'LabelHorizontalAlignment','right');
yline(m_0*g,'b--',sprintf('mg_0=%.0f N',m_0*g),'LineWidth',1.5,'LabelHorizontalAlignment','right');
xline(t_delay,'--','Valve Opened','Color',c_p2,'LineWidth',2,'LabelHorizontalAlignment','left');
text(t_delay/2, F_max*0.5, sprintf('%.0f ms\ndelay',t_delay*1000),...
    'HorizontalAlignment','center','FontSize',9,'Color',c_delay,'FontWeight','bold');
xlabel('Time (s)'); ylabel('Thrust (N)');
title('Thrust vs Time','FontWeight','bold');
xlim([0 t_land*1.12]); ylim([0 F_max*1.18]);
set(ax21,'FontSize',9);

ax22=subplot(3,3,5); hold on; grid on; box on;
fill([0 t_delay_end t_delay_end 0],[0 0 110 110],...
    c_delay,'FaceAlpha',0.12,'EdgeColor','none');
stairs(t_log,thr_log*100,'b-','LineWidth',1.8);
fill([t_log fliplr(t_log)],[thr_log*100 zeros(1,N)],'b','FaceAlpha',0.2,'EdgeColor','none');
yline(hover_duty*100,'g--',sprintf('Hover: %.1f%%',hover_duty*100),'LineWidth',2,'LabelHorizontalAlignment','right');
yline(100,'r--','Full Throttle','LineWidth',1.2);
xline(t_delay,'--','Valve Opened','Color',c_p2,'LineWidth',2,'LabelHorizontalAlignment','left');
text(t_delay/2,50,'CLOSED','HorizontalAlignment','center','FontSize',10,'Color',c_delay,'FontWeight','bold');
xlabel('Time (s)'); ylabel('Main Valve Throttle (%)');
title('Throttle Profile','FontWeight','bold');
xlim([0 t_land*1.12]); ylim([0 110]);
set(ax22,'FontSize',9);

ax23=subplot(3,3,6);
yyaxis left; hold on; grid on; box on;
area(t_log,mg_log,'FaceColor',[0.2 0.6 1],'FaceAlpha',0.45,'EdgeColor','none');
plot(t_log,mg_log,'b-','LineWidth',2);
ylabel('Remaining Propellant (kg)'); ylim([0 m_gas_0*1.12]);
yyaxis right;
plot(t_log,mdot_log,'r-','LineWidth',1.2);
yline(mdot_max,'r--','mdot_{max}','LineWidth',1);
ylabel('Mass Flow Rate (kg/s)'); ylim([0 mdot_max*1.3]);
yline(m_gas_final/m_gas_0,'g--',sprintf('Left: %.1f%%',m_gas_final/m_gas_0*100),'LineWidth',1.5);
xlabel('Time (s)');
title('Propellant Consumption','FontWeight','bold');
xlim([0 t_land*1.12]);
set(ax23,'FontSize',9);

ax31=subplot(3,3,7); hold on; grid on; box on;
plot(t_log,P_log,'b-','LineWidth',1.5);
yline(300,'k--','300 bar','LineWidth',1);
yline(10,'g--','P_{reg}=10 bar','LineWidth',1.5,'LabelHorizontalAlignment','right');
yline(96,'r--','P_{min}=96 bar','LineWidth',1.5,'LabelHorizontalAlignment','right');
xlabel('Time (s)'); ylabel('Tank Pressure (bar)');
title('Tank Pressure vs Time','FontWeight','bold');
xlim([0 t_land*1.12]);
set(ax31,'FontSize',9);

ax32=subplot(3,3,8); hold on; grid on; box on;
area(t_log,m_log,'FaceColor',[0.7 0.7 0.7],'FaceAlpha',0.35,'EdgeColor','none');
plot(t_log,m_log,'k-','LineWidth',2);
yline(m_dry,'r--',sprintf('m_{dry}=%.3f kg',m_dry),'LineWidth',1.5);
yline(m_0,'b--',sprintf('m_0=%.3f kg',m_0),'LineWidth',1.5);
xlabel('Time (s)'); ylabel('Total Mass (kg)');
title('Vehicle Mass vs Time','FontWeight','bold');
xlim([0 t_land*1.12]); ylim([m_dry*0.97 m_0*1.03]);
set(ax32,'FontSize',9);

ax33=subplot(3,3,9); hold on; grid on; box on;
plot(t_log,z3_log,'r-','LineWidth',1.5,'DisplayName','z3 disturbance');
plot(t_log,z2_log-v_log,'b-','LineWidth',1,'DisplayName','z2-v speed error');
yline(0,'k:','LineWidth',1);
xlabel('Time (s)'); ylabel('ESO Output');
title('Extended State Observer','FontWeight','bold');
legend('Location','best','FontSize',8);
xlim([0 t_land*1.12]);
set(ax33,'FontSize',9);

sgtitle(sprintf(['LANDING CONTROLLER v6 - 600N + %.0fms Valve Delay\n'...
    'v_{land} = %.4f m/s | Propellant left: %.1f%% | Duration: %.2f s'],...
    t_delay*1000, abs(v_land), m_gas_final/m_gas_0*100, t_land),...
    'FontSize',13,'FontWeight','bold');

print(fig1,'landing_v6_main.png','-dpng','-r150');
fprintf('Fig1 (main panel) saved.\n');

%% Figure 2: valve timing and speed profile
fig2=figure('Name','Landing Controller v6 - Valves & Speed',...
    'Position',[60 60 1300 900],'Color','w');

ax_v1=subplot(2,2,1); hold on; grid on; box on;
fill([0 t_delay_end t_delay_end 0],[-12 -12 5 5],c_delay,'FaceAlpha',0.15,'EdgeColor','none');
plot(t_log,v_log,'b-','LineWidth',2,'DisplayName','Actual speed');
plot(t_log,vref_log,'r--','LineWidth',2,'DisplayName','v_{ref}');
plot(t_log,z2_log,'g-','LineWidth',1.2,'DisplayName','ESO z2');
yline(-0.2,'k-','Spec 0.2 m/s','LineWidth',2.5,'LabelHorizontalAlignment','right');
yline(0,'k:','LineWidth',1);
xline(t_delay,'--','Valve Opened','Color',c_p2,'LineWidth',2);
text(t_delay/2,-5,sprintf('%.0fms\ndelay',t_delay*1000),'HorizontalAlignment','center',...
    'FontSize',10,'Color',[0.8 0.5 0],'FontWeight','bold');
xlabel('Time (s)'); ylabel('Speed (m/s)');
title('Speed Profile and Reference Tracking','FontWeight','bold');
legend('Location','southeast','FontSize',9);
xlim([0 t_land*1.1]);
set(ax_v1,'FontSize',10);

ax_v2=subplot(2,2,2); hold on; grid on; box on;
if any(idx_p1); plot(h_log(idx_p1),v_log(idx_p1),'.','Color',c_delay,'MarkerSize',2,'DisplayName','Phase1 Delay'); end
if any(idx_p2); plot(h_log(idx_p2),v_log(idx_p2),'.','Color',c_p2,'MarkerSize',2,'DisplayName','Phase2 Full Thrust'); end
if any(idx_p3); plot(h_log(idx_p3),v_log(idx_p3),'.','Color',c_p3,'MarkerSize',2,'DisplayName','Phase3 PD Hover'); end
h_theory=linspace(0.01,h_cmd,400);
v_theory=-sqrt(2*a_net_max*h_theory);
plot(h_theory,v_theory,'k--','LineWidth',2,'DisplayName','Optimal braking curve');
plot(h_cmd,-v_cmd,'r^','MarkerSize',13,'MarkerFaceColor','r','DisplayName','Command Point');
plot(0,v_land,'k*','MarkerSize',13,'LineWidth',2,'DisplayName',sprintf('Landing %.4f m/s',abs(v_land)));
yline(-0.2,'g-','Spec','LineWidth',2);
xlabel('Altitude (m)'); ylabel('Speed (m/s)');
title('Altitude-Speed Phase Plane','FontWeight','bold');
legend('Location','southwest','FontSize',8);
set(ax_v2,'FontSize',10);

ax_v3=subplot(2,2,3); hold on; grid on; box on;
fill([0 t_delay t_delay 0],[0 0 1.1 1.1],c_delay,'FaceAlpha',0.25,'EdgeColor','none');
stairs(t_log,thr_log,'b-','LineWidth',2,'DisplayName','Main valve');
balance_valve=double(phase_log>=3)*0.5;
stairs(t_log,balance_valve,'r-','LineWidth',1.5,'DisplayName','Balance valve');
yline(hover_duty,'g--',sprintf('Hover=%.3f',hover_duty),'LineWidth',1.5,'LabelHorizontalAlignment','right');
xline(t_delay,'--','Color',c_p2,'LineWidth',2);
text(t_delay/2, 0.55, sprintf('%.0f ms\nCLOSED',t_delay*1000),...
    'HorizontalAlignment','center','FontSize',11,'Color',[0.8 0.5 0],'FontWeight','bold');
text(t_delay+0.05, 0.95, 'OPEN','FontSize',11,'Color',c_p2,'FontWeight','bold');
xlabel('Time (s)'); ylabel('Valve Duty Cycle [-]');
title('Valve Timing - 350ms Delay Included','FontWeight','bold');
legend('Location','northeast','FontSize',9);
xlim([0 t_land*1.1]); ylim([0 1.1]);
set(ax_v3,'FontSize',10);

ax_v4=subplot(2,2,4); hold on; grid on; box on;
t_zoom=max(0,t_land-2.0); idx_z=t_log>=t_zoom;
yyaxis left;
plot(t_log(idx_z),h_log(idx_z),'b-','LineWidth',2.5);
yline(h_pd,'b--','PD Hand-off','LineWidth',1.5);
yline(0,'k-','Ground','LineWidth',2);
ylabel('Altitude (m)','Color','b');
ylim([-0.1 max(h_log(idx_z))*1.3]);
yyaxis right;
plot(t_log(idx_z),abs(v_log(idx_z)),'r-','LineWidth',2.5);
yline(0.2,'g-','Spec 0.2','LineWidth',2.5);
yline(abs(v_ref_pd),'b--',sprintf('Target %.2f',abs(v_ref_pd)),'LineWidth',1.5);
ylabel('|Speed| (m/s)','Color','r');
ylim([0 max(abs(v_log(idx_z)))*1.35]);
xlabel('Time (s)');
title('Final Approach - Last 2s','FontWeight','bold');
text(t_zoom+0.05,max(abs(v_log(idx_z)))*0.85,...
    sprintf('v_{land}=%.4f m/s',abs(v_land)),...
    'FontSize',13,'FontWeight','bold','Color',result_color);
set(ax_v4,'FontSize',10);

sgtitle(sprintf('LANDING CONTROLLER v6 - Valve Timing & %.0fms Delay Analysis',t_delay*1000),...
    'FontSize',12,'FontWeight','bold');

print(fig2,'landing_v6_valves.png','-dpng','-r150');
fprintf('Fig2 (valves & speed) saved.\n');

%% Figure 3: energy and system analysis
fig3=figure('Name','Landing Controller v6 - Energy & System',...
    'Position',[90 90 1300 900],'Color','w');

ax_e1=subplot(2,2,1); hold on; grid on; box on;
KE=0.5*m_log.*v_log.^2;
PE=m_log*g.*h_log;
TE=KE+PE;
plot(t_log,KE,'r-','LineWidth',1.5,'DisplayName','Kinetic (KE)');
plot(t_log,PE,'b-','LineWidth',1.5,'DisplayName','Potential (PE)');
plot(t_log,TE,'k-','LineWidth',2,'DisplayName','Total (TE)');
xline(t_delay,'--','Valve Opened','Color',c_p2,'LineWidth',1.5);
xlabel('Time (s)'); ylabel('Energy (J)');
title('Mechanical Energy Breakdown','FontWeight','bold');
legend('Location','best','FontSize',9);
xlim([0 t_land*1.1]);
set(ax_e1,'FontSize',10);

ax_e2=subplot(2,2,2); hold on; grid on; box on;
TWR_inst=F_log./(m_log*g);
plot(t_log,TWR_inst,'b-','LineWidth',2);
yline(1.0,'r--','TWR=1 (hover)','LineWidth',2,'LabelHorizontalAlignment','right');
yline(TWR,'g--',sprintf('TWR_{max}=%.3f',TWR),'LineWidth',1.5,'LabelHorizontalAlignment','right');
yline(hover_duty,'m--',sprintf('Hover duty=%.3f',hover_duty),'LineWidth',1.2);
fill([t_log fliplr(t_log)],[TWR_inst ones(1,N)],[0.5 0.8 1],'FaceAlpha',0.2,'EdgeColor','none');
xline(t_delay,'--','Valve Opened','Color',c_p2,'LineWidth',1.5);
xlabel('Time (s)'); ylabel('TWR');
title('Instantaneous Thrust/Weight Ratio','FontWeight','bold');
xlim([0 t_land*1.1]); ylim([0 TWR*1.3]);
set(ax_e2,'FontSize',10);

ax_e3=subplot(2,2,3); hold on; grid on; box on;
gas_pct=mg_log/m_gas_0*100;
used_pct=100-gas_pct;
area(t_log,100,'FaceColor',[0.9 0.9 0.9],'FaceAlpha',0.5,'EdgeColor','none');
area(t_log,used_pct,'FaceColor',[1 0.4 0.1],'FaceAlpha',0.6,'EdgeColor','none');
plot(t_log,gas_pct,'b-','LineWidth',2.5,'DisplayName','Remaining propellant');
yline(m_gas_final/m_gas_0*100,'g--',...
    sprintf('Left at landing: %.1f%%',m_gas_final/m_gas_0*100),'LineWidth',2);
plot(t_land,m_gas_final/m_gas_0*100,'g*','MarkerSize',12,'LineWidth',2);
xline(t_delay,'--','Valve Opened','Color',c_p2,'LineWidth',1.5);
yline(5.7,'k:','Previous version: 5.7%','LineWidth',1.5,'LabelHorizontalAlignment','right');
xlabel('Time (s)'); ylabel('Propellant (%)');
title('Propellant Efficiency (Delay-Predictive)','FontWeight','bold');
xlim([0 t_land*1.1]); ylim([0 105]);
set(ax_e3,'FontSize',10);

ax_e4=subplot(2,2,4); axis off;
if abs(v_land)<=0.2; s_str='MET'; else; s_str='NOT MET'; end

summary=sprintf(['  LANDING CONTROLLER v6 - DELAY-PREDICTIVE\n'...
    '  ================================\n'...
    '  Mass          : %.3f kg\n'...
    '  Max thrust    : %.0f N\n'...
    '  TWR           : %.3f\n'...
    '  Valve delay   : %.0f ms\n'...
    '  ================================\n'...
    '  TRIGGER\n'...
    '  ================================\n'...
    '  Command h     : %.3f m\n'...
    '  Command v     : %.3f m/s\n'...
    '  After delay   : h~3.41m\n'...
    '  PD hand-off h : %.1f m\n'...
    '  ================================\n'...
    '  RESULT\n'...
    '  ================================\n'...
    '  Landing time  : %.3f s\n'...
    '  LANDING SPEED : %.4f m/s\n'...
    '  Propellant left: %.1f%%\n'...
    '  (Previous: 5.7%%)\n'...
    '  Spec          : %s\n'...
    ],m_0,F_max,TWR,t_delay*1000,...
    h_cmd,v_cmd,h_pd,...
    t_land,abs(v_land),m_gas_final/m_gas_0*100,s_str);

text(0.03,0.97,summary,'Units','normalized','VerticalAlignment','top',...
    'FontName','Courier New','FontSize',9.5,'Interpreter','none');
text(0.5,0.13,sprintf('%.4f m/s',abs(v_land)),...
    'Units','normalized','HorizontalAlignment','center',...
    'FontSize',22,'FontWeight','bold','Color',result_color);
text(0.5,0.06,result_str,...
    'Units','normalized','HorizontalAlignment','center',...
    'FontSize',12,'FontWeight','bold','Color',result_color);

text(0.5,0.22,sprintf('Propellant savings: %.1f%% -> %.1f%%',...
    5.7, m_gas_final/m_gas_0*100),...
    'Units','normalized','HorizontalAlignment','center',...
    'FontSize',11,'Color',[0 0.5 0]);

sgtitle('LANDING CONTROLLER v6 - Energy & System Analysis (Delay Included)',...
    'FontSize',12,'FontWeight','bold');

print(fig3,'landing_v6_energy.png','-dpng','-r150');
fprintf('Fig3 (energy & system) saved.\n');

fprintf('\nAll figures generated:\n');
fprintf('  landing_v6_main.png\n');
fprintf('  landing_v6_valves.png\n');
fprintf('  landing_v6_energy.png\n');
fprintf('=================================================\n');
fprintf('Propellant savings: %.1f%% -> %.1f%%\n', 5.7, m_gas_final/m_gas_0*100);
fprintf('=================================================\n');
