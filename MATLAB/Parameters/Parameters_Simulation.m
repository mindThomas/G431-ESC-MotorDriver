% Sensor Noise parameters
EnableNoise = false;
EnableADCquantization = true;

% Specify noise variance for simulated sensors (can be the same as used in estimator)     
sensor_sigma2_encoder = 0.5;
sensor_sigma2_current_sense = (0.5/3)^2; % 3 sigma = 0.5 A
sensor_sigma2_backemf = (0.1/3)^2; % 3 sigma = 0.1 V

% PWM
PWM_Duty = 0.5;

% Initial conditions
theta0 = 0;
dTheta0 = 0 * 2*pi; % 2 RPS
I0 = 0;