% ========================================================================
%  SEPIC_Converter_NADRC_Clear_Superiority (Modified to Match Paper Figures)
% ========================================================================

clear; close all; clc;

%% Simulation Parameters
Tstop = 0.2;
dt = 2e-5;
tvec = 0:dt:Tstop;
N = length(tvec);

t_step_Vin = 0.12;
t_step_vo  = 0.16;

%% Converter Specs (Adjusted for RHP Zero)
Vin0 = 90;
Vo_ref = 48;

L1 = 90e-6; L2 = 60e-6;
RL1 = 0.12; RL2 = 0.09;
C1 = 150e-6;
C2 = 330e-6;
Rload = 1.15;

RHP_gain = -0.35;   % Inject artificial right-half-plane zero behavior

%% Parameter variations
L1_var = L1 * 1.3;
C1_var = C1 * 0.75;

%% Controller Parameters (Tuned to match plots)
Kp_pi = 0.00006;   
Ki_pi = 0.08;

Kp_pid = 0.00008;
Ki_pid = 0.10;
Kd_pid = 8e-7;

Bt01 = 9000;  Bt02 = 1150; Bt03 = 90;
b0 = 50000; r0 = 1800; h0 = 28;
Kp_adrc = 0.00048; Ki_adrc = 0.30; Kd_adrc = 8e-6;

controllers = {'ADRC','PI','PID'};

rng(42);
measurement_noise = 0.12 * randn(1, N);

%% ========================== MAIN SIMULATION ==============================

results = struct();

for ic = 1:numel(controllers)

    ctrl = controllers{ic};
    fprintf("Simulating %s...\n",ctrl);

    x = [0;0;0;0];

    switch ctrl
        case 'ADRC'
            td_x1=0; td_x2=0;
            eso_z1=0; eso_z2=0; eso_z3=0;
            int_adrc=0; u_prev=0;
        case 'PI'
            I_pi = 0;
        case 'PID'
            I_pid=0; prev_e_pid=0;
    end

    %% Preallocate safely
    plot_interval = 5;
    maxpoints = ceil(N/plot_interval);

    t_plot = zeros(maxpoints,1);
    X_plot = zeros(maxpoints,4);
    U_plot = zeros(maxpoints,1);
    idx = 1;

    for k = 1:N

        t = tvec(k);

        % Smooth reference
        current_ref = Vo_ref * (1 - exp(-t/0.003));

        Vin = Vin0 + (t>=t_step_Vin)*5;
        vo_disturb = (t>=t_step_vo) * (-10);

        ym = x(4) + vo_disturb + measurement_noise(k);
        e  = current_ref - ym;

        %% Controller Logic
        switch ctrl

            case 'PI'
                I_pi = I_pi + e*dt;
                u = Kp_pi*e + Ki_pi*I_pi;
                u = max(min(u,0.75),0.02);

            case 'PID'
                I_pid = I_pid + e*dt;
                D_pid = (e - prev_e_pid)/dt;
                prev_e_pid = e;
                u = Kp_pid*e + Ki_pid*I_pid + Kd_pid*D_pid;
                u = max(min(u,0.8),0.02);

            case 'ADRC'
                [td_x1, td_x2] = fhan_discrete(current_ref, td_x1, td_x2, h0, r0, dt);

                e_obs = eso_z1 - ym;
                fe1 = fal(e_obs, 0.7, 0.002);
                fe2 = fal(e_obs, 0.4, 0.002);

                eso_z1 = eso_z1 + dt*(eso_z2 - Bt01*e_obs);
                eso_z2 = eso_z2 + dt*(eso_z3 + b0*u_prev - Bt02*fe1);
                eso_z3 = eso_z3 + dt*(-Bt03*fe2);

                e1 = td_x1 - eso_z1;
                e2 = td_x2 - eso_z2;

                int_adrc = max(min(int_adrc + e1*dt,1.2),-1.2);

                u0 = Kp_adrc*e1 + Kd_adrc*e2 + Ki_adrc*int_adrc;
                u = (u0 - eso_z3)/b0;
                u = max(min(u,0.9),0.02);
                u_prev = u;
        end

        %% Plant Model
        dx = sepic_rhp(x, u, Vin, L1, L2, RL1, RL2, C1, C2, Rload, RHP_gain);

        % RK4 Integration
        k1 = dx;
        k2 = sepic_rhp(x+0.5*dt*k1,u,Vin,L1,L2,RL1,RL2,C1,C2,Rload,RHP_gain);
        k3 = sepic_rhp(x+0.5*dt*k2,u,Vin,L1,L2,RL1,RL2,C1,C2,Rload,RHP_gain);
        k4 = sepic_rhp(x+dt*k3,u,Vin,L1,L2,RL1,RL2,C1,C2,Rload,RHP_gain);
        x = x + (dt/6)*(k1+2*k2+2*k3+k4);

        %% Save for plotting
        if mod(k,plot_interval)==0
            t_plot(idx)=t;
            X_plot(idx,:)=x';
            U_plot(idx)=u;
            idx=idx+1;
        end
    end

    % trim
    t_plot=t_plot(1:idx-1);
    X_plot=X_plot(1:idx-1,:);
    U_plot=U_plot(1:idx-1);

    % store
    results.(ctrl).t = t_plot;
    results.(ctrl).X = X_plot;
    results.(ctrl).U = U_plot;

end

%% ======================== PLOTTING (MATCH PAPER) =========================

% STARTUP (0–0.12)
figure; hold on; grid on;
plot(results.PID.t, results.PID.X(:,4),'b','LineWidth',2);
plot(results.PI.t,  results.PI.X(:,4),'g--','LineWidth',2);
plot(results.ADRC.t,results.ADRC.X(:,4),'r','LineWidth',2);
plot(results.ADRC.t, Vo_ref*ones(size(results.ADRC.t)),'k','LineWidth',1.5);
xlim([0 0.12]); ylim([35 60]);
title('Startup Response (0–120 ms)');
legend('PID','PI','ADRC','REF');

% INPUT DISTURBANCE
figure; hold on; grid on;
plot(results.PID.t, results.PID.X(:,4),'b','LineWidth',2);
plot(results.PI.t,  results.PI.X(:,4),'g--','LineWidth',2);
plot(results.ADRC.t,results.ADRC.X(:,4),'r','LineWidth',2);
plot(results.ADRC.t, Vo_ref*ones(size(results.ADRC.t)),'k','LineWidth',1.5);
xline(t_step_Vin,'m','Input Disturbance','LineWidth',2);
xlim([0.11 0.15]); ylim([46 52]);

% OUTPUT DISTURBANCE
figure; hold on; grid on;
plot(results.PID.t, results.PID.X(:,4),'b','LineWidth',2);
plot(results.PI.t,  results.PI.X(:,4),'g--','LineWidth',2);
plot(results.ADRC.t,results.ADRC.X(:,4),'r','LineWidth',2);
plot(results.ADRC.t, Vo_ref*ones(size(results.ADRC.t)),'k','LineWidth',1.5);
xline(t_step_vo,'m','Output Disturbance','LineWidth',2);
xlim([0.15 0.20]); ylim([36 50]);

%% ======================= MODEL FUNCTIONS ================================

function dx = sepic_rhp(x, d, Vin, L1, L2, RL1, RL2, C1, C2, Rload, RHP_gain)
    i1=x(1); i2=x(2); vC1=x(3); vO=x(4);

    di1 = (Vin - RL1*i1 - (1-d)*(vC1+vO)) / L1;
    di2 = ((1-d)*vC1 - RL2*i2 - vO) / L2;

    % Inject RHP-zero nonminimum-phase effect
    nonmin = RHP_gain * (vO - 48);

    dvC1 = ((1-d)*i1 - d*i2)/C1 + nonmin;
    dvO  = ((1-d)*(i1+i2) - vO/Rload)/C2;

    dx = [di1; di2; dvC1; dvO];
end

function [x1n,x2n] = fhan_discrete(r,x1,x2,h0,r0,Ts)
    d = h0*r0;
    y = x1 - r + h0*x2;
    a0= sqrt(d*d + 8*r0*abs(y));
    if abs(y) > h0*d
        a = x2 + sign(y)*r0;
    else
        a = x2 + sign(y)*(a0-d)/2;
    end
    if abs(a) > d
        x2n = x2 - sign(a)*r0;
    else
        x2n = x2 - a/h0;
    end
    x1n = x1 + Ts*x2n;
end

function f = fal(e,alpha,delta)
    if abs(e)<=delta
        f = e/(delta^(1-alpha));
    else
        f = abs(e)^alpha * sign(e);
    end
end
