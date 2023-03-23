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
G = Kt / (B + J*s);

% Current filter
f_LPF = SpeedController_SpeedLPF_Freq;
w = 2*pi * f_LPF;
H_LPF = w / (s + w);

% Controller parameters
fs = SpeedController_Freq; % controller frequency [Hz]
Ts = 1/fs;

K = 0.41;
Ti = B/J;
Kp = K;
Ki = K * Ti;

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
CL = D*G / (1 + D*G*P*H_LPF);
uCL = 1 - P*H_LPF*CL;

figure(1);
pzmap(OL);

figure(2);
subplot(1,2,1);
margin(OL); setoptions(gcr, 'FreqUnits', 'Hz'); grid on;
subplot(2,2,2);
rlocus(OL);
subplot(2,2,4);
step(CL);
hold on;
step(uCL);
hold off;
legend('Output (speed)', 'Input (current)');

% Compute settling time
[Gm,Pm,Wcg,Wcp] = margin(OL)
phaseMargin = Pm
crossoverFreq = Wcp / (2*pi)
bandwidthFreq = 2*crossoverFreq
omega_n = 2*pi*bandwidthFreq / 1.4
riseTime = 1.8 / omega_n
zeta = Pm/100
settlingTime = -log(0.01 * sqrt(1-zeta^2)) / (zeta*omega_n)

%% Use PID tuner to get optimal gains
[C_pi, info] = pidtune(G * P * H_LPF, 'PI')
K = C_pi.Kp
Ti = C_pi.Ki / K
Kp = K;
Ki = K * Ti;

%% Launch interactive Control System Designer
controlSystemDesigner({'rlocus', 'bode'}, G, D, Feedback, 1);
