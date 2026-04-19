# Design-Control-of-High-Speed-BLDC-Motor
Developed a MATLAB-based control model using real industrial datasets, implementing PI/PID controllers with variable tuning for optimized system performance.

% ================= SYSTEM LIMITS =================
V_supply = 48;         
RPM_MAX = 50000;       

% ================= REFERENCE =================
N_ref = 50000;
omega_ref = 2*pi*N_ref/60;

% ================= MOTOR PARAMETERS =================
Ke = 0.0075;      
Kt = 0.0075;      
R_phase = 1;   

J = 1.2e-5;        
B = 7e-6;        % Friction
T_load = 0.004;  

I_max = 12;      % Current limit

%% ================= PARAMETER RANGES (CALCULATED) =================
% Ke (Back EMF Constant)
% Min: 0.0068   Max: 0.0075

% Kt (Torque Constant)
% Min: 0.0068   Max: 0.0075

% R_phase (Phase Resistance)
% Min: 0.45 Ω   Max: 1 Ω

% J (Rotor Inertia)
% Min: 5e-6     Max: 2e-5

% T_load (Load Torque)
% Min: 0.0015   Max: 0.004

% B (Friction)
% Min: 4e-6     Max: 8e-6

%% ================= CONTROLLER =================
Kp = 1.5;
Ki = 150;
Kaw = 50;

%% ================= SIMULATION =================
dt = 1e-4;
t_end = 2;
t = 0:dt:t_end;

omega = zeros(size(t));
i = zeros(size(t));
V_control = zeros(size(t));
error_int = 0;

%% ================= DISPLAY =================
fprintf('\nTime\tSpeed(RPM)\tVoltage\n');
fprintf('---------------------------------\n');

%% ================= CLOSED LOOP =================
for k = 1:length(t)-1
    
    % Error
    error = omega_ref - omega(k);
    
    % Feedforward
    V_ff = Ke * omega_ref;
    
    % PI controller
    V_pi = Kp*error + Ki*error_int;
    
    % Unsaturated control
    V_unsat = V_ff + V_pi;
    
    % Voltage saturation (NO locking)
    V_control(k) = min(max(V_unsat, 0), V_supply);
    
    % Anti-windup
    error_int = error_int + (error + Kaw*(V_control(k) - V_unsat)) * dt;
    
    % Motor current
    i(k) = (V_control(k) - Ke*omega(k)) / R_phase;
    
    % Current limiting
    if i(k) > I_max
        i(k) = I_max;
    elseif i(k) < 0
        i(k) = 0;
    end
    
    % Electromagnetic torque (NO boost)
    T_e = Kt * i(k);
    
    % Speed dynamics
    domega = (T_e - T_load - B*omega(k)) / J;
    omega(k+1) = omega(k) + domega * dt;
    
    % Speed limit
    if omega(k+1) > omega_ref
        omega(k+1) = omega_ref;
    end
    
    % Live display
    if mod(k,200) == 0
        fprintf('%.2f\t%.0f\t\t%.2f\n', t(k), omega(k)*60/(2*pi), V_control(k));
    end
end

% FIX: Assign last values correctly
i(end) = i(end-1);
V_control(end) = V_control(end-1);

%% ================= RESULTS =================
N = omega * 60/(2*pi);
E = Ke * omega;
T = Kt * i;

P_out = T .* omega;
P_in  = V_control .* i;

eff = (P_out ./ P_in) * 100;
eff(isnan(eff)) = 0;

%% ================= FINAL OUTPUT =================
fprintf('\n========== FINAL RESULTS ==========\n');
fprintf('Reference Speed       : %.0f RPM\n', N_ref);
fprintf('Final Speed           : %.0f RPM\n', N(end));
fprintf('Final Voltage         : %.2f V\n', V_control(end));
fprintf('Final Current         : %.2f A\n', i(end));
fprintf('Back EMF              : %.2f V\n', E(end));

T_final = Kt * i(end);
Pout_final = T_final * omega(end);
Pin_final  = V_control(end) * i(end);

fprintf('Torque                : %.4f Nm\n', T_final);
fprintf('Mechanical Power Out  : %.2f W\n', Pout_final);
fprintf('Electrical Power In   : %.2f W\n', Pin_final);

% Efficiency calculated from actual values
if Pin_final > 0
    eff_final = (Pout_final / Pin_final) * 100;
else
    eff_final = 0;
end

fprintf('Efficiency            : %.2f %%\n', eff_final);
fprintf('===================================\n');

%% ================= PLOTS =================
figure('Name','Closed Loop BLDC Motor','NumberTitle','off');

subplot(3,2,1);
plot(t, N, 'LineWidth',1.5);
xlabel('Time (s)');
ylabel('Speed (RPM)');
title('Speed vs Time');
grid on;

subplot(3,2,2);
plot(t, T, 'LineWidth',1.5);
xlabel('Time (s)');
ylabel('Torque (Nm)');
title('Torque vs Time');
grid on;

subplot(3,2,3);
plot(t, E, 'LineWidth',1.5);
xlabel('Time (s)');
ylabel('Back EMF (V)');
title('Back EMF vs Time');
grid on;

subplot(3,2,4);
plot(t, eff, 'LineWidth',1.5);
xlabel('Time (s)');
ylabel('Efficiency (%)');
title('Efficiency vs Time');
ylim([0 100]);
grid on;

subplot(3,2,5);
plot(t, V_control, 'LineWidth',1.5);
xlabel('Time (s)');
ylabel('Voltage (V)');
title('Controller Voltage vs Time');
grid on;
