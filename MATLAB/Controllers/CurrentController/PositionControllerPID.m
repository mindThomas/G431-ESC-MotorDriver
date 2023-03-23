clear all;
scriptDir = fileparts(mfilename('fullpath'));
addpath(fullfile(scriptDir, '../../Parameters'));
Parameters_Model
Parameters_Controller
Parameters_Estimators

s = tf('s');

%%
% Motor (plant) model (from i to omega)
% See Mechanical Model in ModelLaplace.m
Gspeed = Kt / (B + J*s);
G = 1/s * Gspeed; % add integrator to convert speed to position

% Current filter
f_LPF = SpeedController_SpeedLPF_Freq;
w = 2*pi * f_LPF;
H_LPF = w / (s + w);

% Controller parameters
fs = SpeedController_Freq; % controller frequency [Hz]
Ts = 1/fs;

K = 2.3;
Ti = 2270/K;
Kp = K;
Ki = K * Ti;
Kd = 0;

K = 3.21;
Ti = 903.4;

% PI controller
D = K * (s + Ti)/s;

% Pade delay (sample approximation)
P = (4/Ts - s) / (4/Ts + s);

% Open loop and direct term
Direct = D * G; % is it correct to say "(D+FF) * G"
Feedback = P * H_LPF;
OL = Direct * Feedback;

% Closed loop
%CL = feedback(Direct, Feedback);
% Closed loop with feedforward
CL = (D+FF)*G / (1 + D*G*P*H_LPF);

figure(1);
subplot(1,2,1);
margin(OL); setoptions(gcr, 'FreqUnits', 'Hz'); grid on;
subplot(2,2,2);
rlocus(OL);
subplot(2,2,4);
step(CL);
