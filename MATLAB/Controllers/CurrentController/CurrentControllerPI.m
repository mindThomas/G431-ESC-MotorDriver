clear all;
scriptDir = fileparts(mfilename('fullpath'));
addpath(fullfile(scriptDir, '../../Parameters'));
Parameters_Model
Parameters_Controller
Parameters_Estimators

s = tf('s');

%%
% Motor (plant) model (from u to i)
% See Electrical Model in ModelLaplace.m
G = 1 / (R + L*s);

% Current filter
f_LPF = CurrentController_CurrentLPF_Freq;
w = 2*pi * f_LPF;
H_LPF = w / (s + w);

% Controller parameters
fs = CurrentController_Freq; % controller frequency [Hz]
Ts = 1/fs;

K = 2.0;
Ti = R/L;
Kp = K;
Ki = K * Ti;

% PI controller
D = K * (s + Ti)/s;

% Pade delay (sample approximation)
P = (4/Ts - s) / (4/Ts + s);

% Feedforward gain
FF = 0*R; % inverse DC gain of plant
%FF = 1/(G * H_LPF) * 0.2*R/(L*s + 0.2*R); % inverse plant model

% Open loop and direct term
Direct = D * G; % is it correct to say "(D+FF) * G"
Feedback = P * H_LPF;
OL = Direct * Feedback;

% Closed loop
%CL = feedback(Direct, Feedback);
% Closed loop with feedforward
CL = (D+FF)*G / (1 + D*G*P*H_LPF);
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
legend('Output (current)', 'Input (voltage)');

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

%% Try looptune
% PI = tunablePID('CurrentController','PI')
% Motor = ss(G);
% Motor.InputName = 'voltage';
% Motor.OutputName = 'current';
% 
% Controller = PI * [1, -P * H_LPF];
% Controller.InputName = {'r', 'current'};
% Controller.OutputName = 'voltage';
% 
% [~,C,~,Info] = looptune(Motor, Controller, 1000);
% PIDT = getBlockValue(C,'CurrentController')
% figure; loopview(Motor, Controller, Info)
% 
% T = connect(Motor, C, 'r', 'current');  % closed-loop transfer from r to speed
% figure; step(T)

%% Launch PID Tuner - does not support filter in feedback path to be specified
%pidTuner(G * P * H_LPF, 'PI');

%% Launch interactive Control System Designer
controlSystemDesigner({'rlocus', 'bode'}, G, D,Feedback, 1);

%%
figure(3);
bode(CL); setoptions(gcr, 'FreqUnits', 'Hz'); grid on;

%%
% Compute steady-state error
SSerror = 1 - evalfr(minreal(CL), 0)

bode(CL); setoptions(gcr, 'FreqUnits', 'Hz');

OutputNoiseSupression = feedback(H_LPF, D * G * P);
bode(OutputNoiseSupression); setoptions(gcr, 'FreqUnits', 'Hz'); grid on;

%% Convert to discrete time
Gd = c2d(G, Ts, 'zoh')

% Discrete PI Controller
Dd = c2d(D, Ts, 'tustin')
% u[k]/e[k] = (5.28 - 4.32 z^-1) / (1 - z^-1)
% (1 - z^-1) * u[k] = (5.28 - 4.32 z^-1) * e[k]
% PI controller: u[k] = u[k-1] + 5.28*e[k] - 4.32*e[k-1]
z = tf('z', Ts);
Dd2 = ( Kp*(z-1) + Ki*Ts/2*(z+1) ) / (z - 1);
% Luckily the above are the same :)   (see derivation in OneNote)

Hd_LPF = c2d(H_LPF, Ts, 'tustin');

% Open loop and direct term
Direct = Dd2 * Gd;
Feedback = Hd_LPF;
OL = Direct * Feedback;

FF = 0*R;

% Closed loop
%CL = feedback(Direct, Feedback);
% Closed loop with feedforward
dCL = (Dd2+FF)*Gd / (1 + Dd2*Gd*Hd_LPF); % ref -> output
CL_OutputDisturbance = 1 / (1 + Dd2*Gd*Hd_LPF); % additive disturbance on output -> output
CL_InputDisturbance = Gd / (1 + Dd2*Gd*Hd_LPF); % additive disturbance on plant input -> output

figure(2);
subplot(1,2,1);
margin(OL); setoptions(gcr, 'FreqUnits', 'Hz'); grid on;
%bode(CL); setoptions(gcr, 'FreqUnits', 'Hz'); grid on;
subplot(2,2,2);
rlocus(OL);
subplot(2,2,4);
step(dCL);
hold on;
step(CL);
hold off;

%%
figure(3);
bode(CL); setoptions(gcr, 'FreqUnits', 'Hz'); grid on;


%%
z = tf('z', 1/20000);
MA = 0;
l = 18;
for (i = 0:(l-1))
    MA = MA + z^(-i);
end
MA = MA / l;

figure(4);
bode(MA); setoptions(gcr, 'FreqUnits', 'Hz'); grid on;

%%
[N,Wn,BTA,FILTYPE] = kaiserord([200 600], [1 0], [0.004 0.1], 20000)
B = fir1(N, Wn, FILTYPE, kaiser( N+1,BTA ), 'noscale' )
B = 1/18 * ones(1, 18);
a_coeff = zeros(1, length(B));
a_coeff(1) = 1;
lpf = tf(B, a_coeff, 1/20000)

bode(lpf); setoptions(gcr, 'FreqUnits', 'Hz'); grid on;

f_LPF = 200;
omega_LPF = 2*pi * f_LPF; w = omega_LPF;
H_LPF = omega_LPF / (s + omega_LPF);

%%
hold on;
bode(lpf);
hold off;

%% Downsampled
