clear all;
scriptDir = fileparts(mfilename('fullpath'));
addpath(fullfile(scriptDir, '../Parameters'));
Parameters_Model
Parameters_Simulation

%%
X = [0; R*1.1];
P_prev = eye(2) * 0.01;

dtheta_internal = 0;

i_meas = [0.25, 2.1683;
          0.75, 1.5817];
adc_sample_time = 20e-6; % 20 us

Vin = 9;
duty = 0.2;

%%
[X, P_prev] = MotorCurrentEKF5(X, P_prev, ...
    L, Ke, ... % constants/parameters
    PWM_Frequency, duty, Vin, dtheta_internal, ... % inputs (u)
    i_meas(1,2), i_meas(2,2))  % measurements (z)