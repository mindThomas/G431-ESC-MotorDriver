PCLK = 170e6; % MHz
ADCCLK = 68e6; % MHz
%PCLK = 48e6; % MHz

ADC_Prescaler = 4;
ADC_Clock = PCLK / ADC_Prescaler;

ADC_SampleTimeCycles = 47.5;%640.5; % 41.5;
ADC_SampleTime = (ADC_SampleTimeCycles + 12.5) / ADC_Clock;
fprintf('\nADC Sample time = %2.2fus\n', ADC_SampleTime*1e6);

%% ADC/Duty computation
SampleTime = 16e-6; % us

DutyMax = 1000;
fPWM = 100;

Duty1 = DutyMax*(0.5 - SampleTime*fPWM)
Duty2 = DutyMax*(1.0 - SampleTime*fPWM)


%% Timer frequency calculation
fPWM = 100;
PRESCALER = 32;
TIM_Clock = PCLK / (PRESCALER+1);

ARR = TIM_Clock / fPWM

Duty1 = (ARR+1)*(0.5 - SampleTime*fPWM)
Duty2 = (ARR+1)*(1.0 - SampleTime*fPWM)