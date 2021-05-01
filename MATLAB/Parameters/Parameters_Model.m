% DC Motor parameters
R = 1.8; % [Ohm]
R_off = 1000e6; % 10 MegaOhm when MOSFETs are in off state
L = 5.0e-3; % Motor inductance [Henry]
Kt = 0.155; % Torque constant [Nm/A]
Ke = 0.3878; % Back-EMF constant [V / rad/s]
B = 0*2.5e-3; % Viscous motor friction [Nm / rad/s]

% Gearbox parameters
n = 30; % 1:30 gearing ratio
Bg = 0*1.00e-2; % Viscous gearbox friction [Nm / rad/s] based on output shaft speed
theta_backlash = deg2rad(0.2); % shaft +/- backlash angle/amount

% Inertia parameters
Jm = 2.2 * 10^-4; % 2200 g cm^2
Jw = 1.0 * 10^-4; % 1000 g cm^2
J = Jw + n^2*Jm;

% Encoder parameters
EncoderTicksPrRev = 64; % internal quadrature ticks pr. revolution (before gearing)

% Microprocessor specifics
PWM_Frequency = 10000; % Hz
PWM_Resolution = 100; % steps