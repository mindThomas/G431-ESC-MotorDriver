%% Current controller
CurrentController_Freq = 10000; % controller sample rate
CurrentController_Ts = 1/CurrentController_Freq; 

CurrentController_CurrentLPF_Freq = 500; % Hz
CurrentController_SpeedLPF_Freq = 20; % Hz
CurrentController_VinLPF_Freq = 20; % Hz (used to convert from voltage to duty cycle, currently not considered in CurrentControllerPI.m)

% PID parameters - result of root locus analysis in CurrentControllerPI.m
CurrentController_K = 2.0;
CurrentController_Ti = 1300;  % R/L
CurrentController_Kp = CurrentController_K;
CurrentController_Ki = CurrentController_K * CurrentController_Ti;

%% Speed controller
SpeedController_Freq = 500; % controller sample rate
SpeedController_Ts = 1/SpeedController_Freq; 

SpeedController_SpeedLPF_Freq = 20; % Hz

% PID parameters - result of root locus analysis in SpeedControllerPI.m
SpeedController_K = 0.41;
SpeedController_Ti = 0.5294;  % B/J
SpeedController_Kp = CurrentController_K;
SpeedController_Ki = CurrentController_K * CurrentController_Ti;

%% Position controller
PositionController_Freq = 200; % controller sample rate
PositionController_Ts = 1/SpeedController_Freq; 

% PID parameters - result of root locus analysis in SpeedControllerPI.m
PositionController_K = 0.41;
PositionController_Ti = 0.5294;  % B/J
PositionController_Kp = CurrentController_K;
PositionController_Ki = CurrentController_K * CurrentController_Ti;