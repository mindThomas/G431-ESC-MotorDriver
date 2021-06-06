omega = 0;            
current = 0;
time = 0;
vmot = 0;

dt = 0.01;
Vmot = Vbat;

sigma2_current_sense = 0.0002;
sigma2_omega = 20;
sigma2_vmot = 0.03;

vmot_steady_states = [];
omega_steady_states = [];
current_steady_states = [];
duty_steady_states = [];
time_steady_states = [];


% Brake mode, PWM ON
for (vmot_step = 1:10)
    duty = vmot_step/10;
    Vmot = duty * Vbat;
    for (i = 1:500)
        time(end+1,1) = time(end) + dt;

        exp_t = exp(-1/J*(B + Kt*Ke/R)*dt);
        vmot(end+1,1) = Vmot;
        current(end+1,1) = (Vmot - Ke*omega(end)) / R;
        omega(end+1,1) = omega(end) * exp_t ...
                              + Vmot * Kt/(B*R + Kt*Ke) * (1-exp_t) ...
                              - tau_c * R/(B*R + Kt*Ke) * (1-exp_t);                                                   
        
        if (i > 100)
            time_steady_states(end+1,1) =  time(end,1);
            vmot_steady_states(end+1,1) = Vmot + sqrt(sigma2_vmot)*randn(1,1);
            current_steady_states(end+1,1) = current(end,1) + sqrt(sigma2_current_sense)*randn(1,1);
            omega_steady_states(end+1,1) = omega(end,1) + sqrt(sigma2_omega)*randn(1,1);                          
            duty_steady_states(end+1,1) = duty;
        end
    end
end

idx_coast = length(time);
    
omega0 = omega(end);
t0 = time(end);

% Coast mode, PWM OFF
for (i = 1:200)
    time(end+1,1) = time(end) + dt;

    t = time(end,1) - t0;
    exp_t = exp(-B/J*t);
    current(end+1,1) = 0;    
    omega(end+1,1) = (omega0 + 1/B*tau_c)*exp_t - 1/B*tau_c;
    if (omega(end) < 0)
        omega(end) = 0;
    end
    vmot(end+1,1) = Ke*omega(end,1) + sqrt(sigma2_vmot)*randn(1,1);
end

omega = omega + sqrt(sigma2_omega)*randn(size(omega));

figure(1);
subplot(2,1,1);
plot(time, 60*omega/(2*pi*n_gear));
subplot(2,1,2);
plot(time, current);


%% Load from Simulink (MechanicalParameterEstimation.slx)
time = ramp_data(:,5);
omega = ramp_data(:,1);
current = ramp_data(:,2);
vmot = ramp_data(:,3);
coast = ramp_data(:,4);

vmot_steady_states = vmot(coast > 0 & current > 0);
current_steady_states = current(coast > 0 & current > 0);
omega_steady_states = omega(coast > 0 & current > 0);

idx_coast = find(coast == 0);
idx_coast = idx_coast(1);
omega0 = omega(idx_coast);
t0 = time(idx_coast);

figure(2);
subplot(2,1,1);
plot(time, 60*omega/(2*pi*n_gear));
subplot(2,1,2);
plot(time, current);

%%
kf = JointMotorParamEstimator(1, 0.1);
vmot_prev = 0;
current_out = [];
omega_out = [];
vmot_out = [];
firstTime = true;
duty_change_idx = find(diff(duty_steady_states) > 0.08) + 1;
duty_change_idx(end+1) = length(duty_steady_states);

idx0 = 1;
vmot_steady_states_avg = zeros(size(vmot_steady_states));
current_steady_states_avg = zeros(size(current_steady_states));
omega_steady_states_avg = zeros(size(omega_steady_states));
for (j = 1:length(duty_change_idx))    
    idx1 = duty_change_idx(j);
    
    vmot_avg = RunningAverage;
    current_avg = RunningAverage;
    omega_avg = RunningAverage;
    for (i = idx0:idx1)
        vmot_avg = vmot_avg.Update(vmot_steady_states(i));
        omega_avg = omega_avg.Update(omega_steady_states(i));
        current_avg = current_avg.Update(current_steady_states(i));
    end
    for (i = idx0:idx1)
        vmot_steady_states_avg(i) = vmot_avg.value;
        current_steady_states_avg(i) = current_avg.value;
        omega_steady_states_avg(i) = omega_avg.value;
    end
    
    idx0 = idx1;
end

for (i = 1:length(vmot_steady_states))
    if (~isempty(find(duty_change_idx == i)) || firstTime)        
        kf = kf.Predict(current_steady_states(i), omega_steady_states(i), sigma2_current_sense, sigma2_omega);        
        firstTime = false;
        %vmot_prev = vmot_steady_states(i);
    else
        kf = kf.Update(current_steady_states(i), omega_steady_states(i), vmot_steady_states(i), sigma2_current_sense, sigma2_omega, sigma2_vmot);    
        %vmot_prev = kf.X(1)*kf.X(3) + kf.X(2)*kf.X(4); 
    end        
    %kf.X
    %kf.P
    current_out(end+1,1) = kf.X(3);    
    omega_out(end+1,1) = kf.X(4);
    vmot_out(end+1,1) = kf.X(1)*kf.X(3) + kf.X(2)*kf.X(4);
end
X = kf.X

R
R2 = X(1)
Ke
Ke2 = X(2)

%%
figure(2);
ax1 = subplot(3,1,1);
plot(current_steady_states);
hold on;
plot(current_out);
%plot(current_steady_states_avg);
hold off;

ax2 = subplot(3,1,2);
plot(omega_steady_states);
hold on;
plot(omega_out);
%plot(omega_steady_states_avg);
hold off;

ax3 = subplot(3,1,3);
plot(vmot_steady_states);
hold on;
plot(vmot_out);
%plot(vmot_steady_states_avg);
hold off;
linkaxes([ax1, ax2, ax3], 'x');

%% Least Squares fitting (1)
% Assuming steady state current, which requires the current estimator,
% which in turn requires the resistance and inductance to be known (or
% estimated)
% (Vmot-Ke*omega) / R = i_a
% Vmot = R*i_a + Ke*omega
A = [current_steady_states_avg, omega_steady_states_avg];
y = vmot_steady_states_avg;
X = lscov(A, y);

R
R2 = X(1)
Ke
Ke2 = X(2)


%%
rls = RecursiveLeastSquares_2D(A(1:2,1:2), y(1:2))
for (i = 3:length(A))
    rls = rls.AddMeasurement(A(i,:), y(i));    
end
X = rls.ComputeEstimate()

R
R2 = X(1)
Ke
Ke2 = X(2)


%% MechanicalParameterEKF Test - not working
X = JKeBTauCekf_X_init;
P = JKeBTauCekf_P_init;
for (i = 1:length(vmot_steady_states))
   [X, P, t] = MechanicalParameterEKF(X, P, t, ...
    R, Kt, ... % constants/parameters
    time_steady_states(i), duty_steady_states(i), Vbat, false, ... % inputs (u)
    current_steady_states(i), omega_steady_states(i))  % measurements (z)
    pause;   
end



%% Least Squares fitting (2)
% omega_steadystate = Kt/B*i_a - 1/B*tau_c
A = [current_steady_states_avg, ones(length(current_steady_states_avg), 1)];
y = omega_steady_states_avg;
X = lscov(A, y);

Kt_B = X(1);
B
B2 = Kt / Kt_B
tauc_B = -X(2);
tau_c
tau_c2 = B2 * tauc_B

%% Least Squares fitting (3)
% -B/J*t = log( (omega(t) + 1/B*tau_c) / (omega0 + 1/B*tau_c) )
include_idx = (idx_coast+1:length(omega))';
include_idx(omega(include_idx) <= 0.1) = [];
A = [time(include_idx) - t0];
y =  log( (omega(include_idx) + 1/B*tau_c) / (omega0 + 1/B*tau_c) );
X = lscov(A, y);

B_J = -X(1);
J
J2 = B / B_J

rls = RecursiveLeastSquares(A(1,1), y(1))
for (i = 2:length(A))
    rls = rls.AddMeasurement(A(i,:), y(i));    
end
J2 = -B / rls.ComputeEstimate()

% Manual recursive least squares - should yield same result as above
a_squared = A(1,1)^2;
ay = A(1,1) * y(1);
for (i = 2:length(A))
    a_squared = a_squared + A(i,1)^2;
    ay = ay + A(i,1)*y(i);
end
    
% Compute estimate
x_est = ay / a_squared;
J3 = -B / x_est

%% Least Squares fitting (4)
% Vmot = Ke*omega
include_idx = (idx_coast+1:length(omega))';
include_idx(omega(include_idx) <= 0.1) = [];
A = [omega(include_idx)];
y = vmot(include_idx);
X = lscov(A, y);

Ke
Ke2 = X(1)