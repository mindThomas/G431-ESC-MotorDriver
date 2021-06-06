V_in = 9;
d = 0.5; % PWM_Duty for test
static_test = true; % false = steady state angular velocity
omega = 0; % initial angular velocity
current_sample_offset = 20e-6; % 5 us ADC sample time before the PWM pulse edges
sigma2_current_sense = ((100e-3) / 3)^2; % 3*sigma = 500 mA

%i0 = 1/R * (V_in*(exp(-R/L*t_OFF)-exp(-R/L*dt)) - Ke*omega*(1+exp(-R/L*dt))) / (1 - exp(-R/L*dt));
%i0 = 1

% Dynamic/changing i0
i_ON = @(t,i0,omega) (i0-V_in/R+Ke*omega/R)*exp(-R/L*t) + V_in/R - Ke*omega/R;
i_OFF = @(t,iON,omega) (iON+Ke*omega/R)*exp(-R/L*t) - Ke*omega/R;

% Steady state:
%i_ON2 = @(t) 1/R*( (V_in*(exp(-R/L*t_OFF)-1) - 2*Ke*omega*exp(-R/L*dt))/(1-exp(-R/L*dt)) )*exp(-R/L*t) + V_in/R - Ke*omega/R;
%i_OFF2 = @(t) V_in/R*exp(-R/L*t)*( (1-exp(-R/L*t_ON))/(1-exp(-R/L*dt)) ) - Ke*omega/R*( (1-exp(-R/L*dt)+2*exp(-R/L*dt)*exp(-R/L*t_ON)*exp(-R/L*t))/(1-exp(-R/L*dt)) )

f_sweep = 500:5000;
f_sweep = reshape(repmat(f_sweep, [3,1]), [3*length(f_sweep), 1]);

% Initialize
current_min = 0;
current_max = 0;
current_on = 0;
current_on_sample_location = 0;
current_off = 0;
current_off_sample_location = 0;
for (i = 1:500)
    f = f_sweep(1); % PWM_Frequency    
    dt = 1/f;
    t_ON = d*dt;
    t_OFF = (1-d)*dt;
    
    if (current_sample_offset > t_ON || current_sample_offset > t_OFF)
        error('Sample offset is longer than the ON or OFF time caused by the PWM period. The PWM frequency is too high for this sample offset.');
    end
    
    i_max = i_ON(t_ON, current_min(end), omega(end));
    current_on(1) = i_ON(t_ON-current_sample_offset, current_min(end), omega(end)) + sqrt(sigma2_current_sense)*randn(1,1);
    current_off(1) = i_OFF(t_OFF-current_sample_offset, i_max, omega(end)) + sqrt(sigma2_current_sense)*randn(1,1);
    i_min = i_OFF(t_OFF, i_max, omega(end));
    
    current_max(1) = i_max;
    current_min(1) = i_min;
    
    current_on_sample_location(1) = (t_ON-current_sample_offset) / dt;
    current_off_sample_location(1) = (dt-current_sample_offset) / dt;
    
    exp_t = exp(-1/J*(B + Kt*Ke/R)*dt);
    if (static_test == false)
        omega(end,1) = omega(end) * exp_t ...
                      + d*V_in * Kt/(B*R + Kt*Ke) * (1-exp_t) ...
                      - tau_c * R/(B*R + Kt*Ke) * (1-exp_t);   
    end
end

for (i = 2:length(f_sweep))
    f = f_sweep(i); % PWM_Frequency    
    dt = 1/f;
    t_ON = d*dt;
    t_OFF = (1-d)*dt;
    
    if (current_sample_offset > t_ON || current_sample_offset > t_OFF)
        error('Sample offset is longer than the ON or OFF time caused by the PWM period. The PWM frequency is too high for this sample offset.');
    end
    
    % Iterate to find current steady-state
    i_min = current_min(end);
    for (j = 1:5)
        i_max = i_ON(t_ON, i_min, omega(end));
        i_min = i_OFF(t_OFF, i_max, omega(end));
    end
    
    %i_max = i_ON(t_ON, current_min(end));
    i_max = i_ON(t_ON, i_min, omega(end));
    current_on(end+1) = i_ON(t_ON-current_sample_offset, current_min(end), omega(end)) + sqrt(sigma2_current_sense)*randn(1,1);
    current_off(end+1) = i_OFF(t_OFF-current_sample_offset, i_max, omega(end)) + sqrt(sigma2_current_sense)*randn(1,1);
    i_min = i_OFF(t_OFF, i_max, omega(end));
    
    current_max(end+1) = i_max;
    current_min(end+1) = i_min;
    
    current_on_sample_location(end+1) = (t_ON-current_sample_offset) / dt;
    current_off_sample_location(end+1) = (dt-current_sample_offset) / dt;
    
%     exp_t = exp(-1/J*(B + Kt*Ke/R)*dt);
%     omega(end+1,1) = omega(end) * exp_t ...
%                       + d*V_in * Kt/(B*R + Kt*Ke) * (1-exp_t) ...
%                       - tau_c * R/(B*R + Kt*Ke) * (1-exp_t);    
end

figure(1);
plot(f_sweep, current_min)
hold on;
plot(f_sweep, current_max)
plot(f_sweep, current_on)
plot(f_sweep, current_off)
hold off;

xlabel('Frequency [Hz]');
ylabel('Current [A]');
legend('MIN', 'MAX', 'ON-END sample', 'OFF-END sample');
title('Current samples during PWM frequency sweep with 50% duty cycle');


% Test RL_EKF2
X = RLekf_X_init;
P = RLekf_P_init;

for (i = 1:length(current_on))
    [X, P, i_avg] = RL_EKF2(X, P, ...
        Ke, ... % constants/parameters
        f_sweep(i), d, V_in, omega, ... % inputs (u)
        current_on(i), current_on_sample_location(i), ...
        current_off(i), current_off_sample_location(i));  % measurements (z)
end

R
R2 = X(2)

L*1e3
L2 = X(3)*1e3

%% Test RL_EKF3
X = [RLekf_X_init(2); RLekf_X_init(3)];
P = RLekf_P_init([2,3],[2,3]);

for (i = 1:length(current_on))
    [X, P, i_avg] = RL_EKF3(X, P, ...
        Ke, ... % constants/parameters
        f_sweep(i), d, V_in, omega, ... % inputs (u)
        current_on(i), current_on_sample_location(i), ...
        current_off(i), current_off_sample_location(i));  % measurements (z)
end

R
R2 = X(1)

L*1e3
L2 = X(2)*1e3


%% Run RL_EKF on captured data from CSV
data = LoadDump('C:\Users\Thomas\Documents\motordriver-python\', '2020-05-10_23-11-47-635.csv');
idx = find(data.duty_cycle > 0 & data.timer_frequency >= 200 & data.timer_frequency <= 5000);
data = Extract(data, idx(1), idx(end));
data.RPM = mean(data.RPM) * ones(size(data.RPM));

figure(2);
ax1 = subplot(2,1,1);
plot(data.time, data.vin);
ax2 = subplot(2,1,2);
plot(data.time, data.current_on);
%plot(data.current_on);
hold on;
plot(data.time, data.current_off);
%plot(data.current_off);
hold off;
legend('Current ON', 'Current OFF');
linkaxes([ax1, ax2], 'x');

figure(3);
ax1 = subplot(2,1,1);
plot(data.time, data.duty_cycle);
ax2 = subplot(2,1,2);
plot(data.time, data.trigger_on);
hold on;
plot(data.time, data.trigger_off);
hold off;
legend('Trigger ON', 'Trigger OFF');
linkaxes([ax1, ax2], 'x');

figure(4);
plot(data.current_on);
hold on;
plot(data.current_off);
hold off;
legend('Current ON', 'Current OFF');

figure(100);
plot(data.timer_frequency, data.current_on, '.');
hold on;
plot(data.timer_frequency, data.current_off, '.');
hold off;
legend('Current ON', 'Current OFF');


%% Test RL_EKF2
X = RLekf_X_init;
P = RLekf_P_init;
data.i_avg = zeros(length(data.time),1);

for (i = 1:length(data.time))
    [X, P, i_avg] = RL_EKF2(X, P, ...
        Ke, ... % constants/parameters
        data.timer_frequency(i), data.duty_cycle(i), data.vin(i), 2*pi/60*data.RPM(i), ... % inputs (u)
        data.current_on(i), data.trigger_on(i), ...
        data.current_off(i), data.trigger_off(i));  % measurements (z)
    data.i_avg(i) = i_avg;    
end

R
R2 = X(2)

L*1e3
L2 = X(3)*1e3
L2 = L2/1e3;

%% Test RL_EKF3
X = [RLekf_X_init(2); RLekf_X_init(3)];
P = RLekf_P_init([2,3],[2,3]);
data.i_avg = zeros(length(data.time),1);

for (i = 1:length(data.time))
    [X, P, i_avg] = RL_EKF3(X, P, ...
        Ke, ... % constants/parameters
        data.timer_frequency(i), data.duty_cycle(i), data.vin(i), 2*pi/60*data.RPM(i), ... % inputs (u)
        data.current_on(i), data.trigger_on(i), ...
        data.current_off(i), data.trigger_off(i));  % measurements (z)
    data.i_avg(i) = i_avg;
end

R
R2 = X(1)

L*1e3
L2 = X(2)*1e3
L2 = L2/1e3;

%% Test L_EKF
X = [RLekf_X_init(1); RLekf_X_init(3)];
P = RLekf_P_init([1,3],[1,3]);
data.i_avg = zeros(length(data.time),1);

for (i = 1:length(data.time))
    [X, P, i_avg] = L_EKF(X, P, ...
        R, Ke, ... % constants/parameters
        data.timer_frequency(i), data.duty_cycle(i), data.vin(i), 2*pi/60*data.RPM(i), ... % inputs (u)
        data.current_on(i), data.trigger_on(i), ...
        data.current_off(i), data.trigger_off(i));  % measurements (z)
    data.i_avg(i) = i_avg;
end

L*1e3
L2 = X(2)*1e3
L2 = L2/1e3;


%% Predict based on new parameters
%R2 = R;
%L2 = L;
% Dynamic/changing i0
%i_ON = @(t,i0,omega,vin) (i0-vin/R2+Ke*omega/R2)*exp(-R2/L2*t) + vin/R2 - Ke*omega/R2;
%i_OFF = @(t,iON,omega) (iON+Ke*omega/R2)*exp(-R2/L2*t) - Ke*omega/R2;

% Steady state:
i_ON = @(t_SAMPLE,t_ON,t_OFF,omega,vin) 1/R2*( (vin*(exp(-R2/L2*t_OFF)-1) - 2*Ke*omega*exp(-R2/L2*(t_ON+t_OFF)))/(1-exp(-R2/L2*(t_ON+t_OFF))) )*exp(-R2/L2*t_SAMPLE) + vin/R2 - Ke*omega/R2;
i_OFF = @(t_SAMPLE,t_ON,t_OFF,omega,vin) vin/R2*exp(-R2/L2*t_SAMPLE)*( (1-exp(-R2/L2*t_ON))/(1-exp(-R2/L2*(t_ON+t_OFF))) ) - Ke*omega/R2*( (1-exp(-R2/L2*(t_ON+t_OFF))+2*exp(-R2/L2*(t_ON+t_OFF))*exp(-R2/L2*t_ON)*exp(-R2/L2*t_SAMPLE))/(1-exp(-R2/L2*(t_ON+t_OFF))) );

i0 = 0;
predict.iON = zeros(length(data.time),1);
predict.iOFF = zeros(length(data.time),1);
for (i = 1:length(data.time))
    t_ON = data.duty_cycle(i) / data.timer_frequency(i);
    t_OFF = (1-data.duty_cycle(i)) / data.timer_frequency(i);
    t_SAMPLE_ON = data.trigger_on(i) / data.timer_frequency(i);
    t_SAMPLE_OFF = data.trigger_off(i) / data.timer_frequency(i) - t_ON;    
    
    %predict.iON(i) = i_ON(t_SAMPLE_ON, i0, 2*pi/60*data.RPM(i), data.vin(i));
    %i_max = i_ON(t_ON, i0, 2*pi/60*data.RPM(i), data.vin(i));
    
    %predict.iOFF(i) = i_OFF(t_SAMPLE_OFF, i_max, 2*pi/60*data.RPM(i));
    %i0 = i_OFF(t_OFF, i_max, 2*pi/60*data.RPM(i));
    
    predict.iON(i) = i_ON(t_SAMPLE_ON, t_ON, t_OFF, 2*pi/60*data.RPM(i), data.vin(i));
    predict.iOFF(i) = i_OFF(t_SAMPLE_ON, t_ON, t_OFF, 2*pi/60*data.RPM(i), data.vin(i));
end


figure(5);
ax1 = subplot(2,1,1);
plot(data.time, data.vin);
ax2 = subplot(2,1,2);
plot(data.time, data.current_on);
hold on;
plot(data.time, data.current_off);
plot(data.time, data.i_avg);

plot(data.time, predict.iON, '--');
plot(data.time, predict.iOFF, '--');
hold off;
legend('Current ON', 'Current OFF', 'Current Average', 'Predict Current ON', 'Predict Current OFF');
linkaxes([ax1, ax2], 'x');

figure(6);
plot(data.timer_frequency, data.current_on, '.');
hold on;
plot(data.timer_frequency, data.current_off, '.');
plot(data.timer_frequency, predict.iON, '.');
plot(data.timer_frequency, predict.iOFF, '.');
hold off;