%% Motor Current estimator
X_init = [0; R*1.01];
P_init = diag([0.01, 0.0005]);

RLekf_X_init = [0; 1; 1e-3]; %[0; R*1.1; L];
RLekf_P_init = diag([1, 0.1, 0.001]);

JKeBTauCekf_X_init = [0; 1e-2; Ke; 1e-6; 1e-3]; % [omega; J; Ke; B; tau_c];
JKeBTauCekf_P_init = diag([0.00001, 0.1, 0.0001, 0.00001, 0.001]);

%% Sample rate
Ts_encoder = 1 / 100; % 100 Hz
Ts_backemf = 1 / 100; % 100 Hz

%% Current sampler
current_sample_end_time_offset = 0e-6; % e.g ADC sample time
Ts_CurrentSampling = 1/PWM_Frequency; % 1 / 10000; % 100 Hz, used for ideal (average) sampler