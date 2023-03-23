% Gearbox parameters
n_gear = 30; % 1:30 gearing ratio
Bg = 0*1.00e-2; % Viscous gearbox friction [Nm / rad/s] based on output shaft speed
theta_backlash = deg2rad(0.2); % shaft +/- backlash angle/amount

% DC Motor parameters
R = 2.6; % [Ohm]
R_off = 1000e6; % 10 MegaOhm when MOSFETs are in off state
L = 0.002; % Motor inductance [Henry]
Kt_AG = 0.157; % Torque constant after gearing [Nm/A]
Ke_AG = 0.276158049818619; % Back-EMF constant after gearing [V / rad/s]
B_AG = 4.77e-4; % Viscous motor friction after/including gearing [Nm / rad/s]
tau_c_AG = 1.1868e-2; % Columb motor friction after/including gearing [N]
Kt_BG = Kt_AG/n_gear; % Torque constant before gearing [Nm/A]
Ke_BG = 0.0095; % Back-EMF constant before gearing [V / rad/s]
B_BG = 6.5369e-7; % Viscous motor friction before/excluding gearing [Nm / rad/s]
tau_c_BG = 3.2405e-4; % Columb motor friction before/excluding gearing [N]

% Inertia parameters
Jm = 1.0 * 10^-6; % 10 g cm^2 Motor inertia (without gearbox), observed before gearing
Jg = 1.0 * 10^-6; % 10 g cm^2 Gearbox inertia, observed after gearing
J_BG = Jm; % Motor inertia without gearbox, before gearing
J_AG = Jg + n_gear^2*Jm; % Motor inertia with gearbox
J_AG = 0.000901; % override with estimated combined inertia [kg*m^2]
J_BG = J_AG / n_gear^2; % override with estimated combined inertia [kg*m^2]

% Parameter selection
Kt = Kt_AG;
Ke = Ke_AG;
B = B_AG;
tau_c = tau_c_AG;
J = J_AG;

% Encoder parameters
EncoderTicksPrRev_BG = 64; % internal quadrature ticks pr. revolution (before gearing)
EncoderTicksPrRev_AG = n_gear * EncoderTicksPrRev_BG; % quadrature ticks pr. revolution (after gearing)
TicksPrRev = EncoderTicksPrRev_AG;

% Battery parameters
Vbat = 12; % Battery voltage [V]

% ADC parameters
ADC_Resolution = 2^12; % 12-bit
ADC_Vref = 3.3;
BackEMF_BigRange_VoltageDivider = 2.2 / (2.2 + 10);
BackEMF_BAT30_ForwardVoltage = 0.1;
Vbus_VoltageDivider = 18 / (18 + 169);

% Current sense parameters
CurrentSense_PGA = 16; % OpAmp programmable gain
CurrentSense_R1 = 1500;
CurrentSense_R2 = 22000;
CurrentSense_R3 = 2200;
CurrentSense_Rsense = 0.015; % 15 mOhm
CurrentSense_G = 1/(CurrentSense_R1 * (1/CurrentSense_R1 + 1/CurrentSense_R2 + 1/CurrentSense_R3)) * CurrentSense_PGA; % See "Op amp calculation.xmcd"
CurrentSense_Voffset = ADC_Vref/(CurrentSense_R2 * (1/CurrentSense_R1 + 1/CurrentSense_R2 + 1/CurrentSense_R3)) * CurrentSense_PGA; % See "Op amp calculation.xmcd"
% Current = (SenseVoltage - Voffset) / (G * Rsense);
% Current = a*SenseVoltage + b
% Where
%    a = 1/(G * Rsense)
%    b = -Voffset/(G * Rsense)

% Microprocessor specifics
PWM_Frequency = 20000; % Hz
PWM_Resolution = 100; % steps