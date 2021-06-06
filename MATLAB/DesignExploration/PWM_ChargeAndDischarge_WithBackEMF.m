f = 20000; % PWM_Frequency
d = 0.2; % PWM_Duty
dt = 1/f;
t_ON = d*dt;
t_OFF = (1-d)*dt;
V_in = 9;
omega = 3;

i0 = 1/R * (V_in*(exp(-R/L*t_OFF)-exp(-R/L*dt)) - Ke*omega*(1-exp(-R/L*dt))) / (1 - exp(-R/L*dt));
%i0 = 1

i_ON = @(t) (i0-V_in/R+Ke*omega/R)*exp(-R/L*t) + V_in/R - Ke*omega/R;
%i_OFF = @(t) (i_ON(t_ON)+Ke*omega/R)*exp(-R/L*t) - Ke*omega/R;
i_OFF = @(t) ( (i0-V_in/R+Ke*omega/R)*exp(-R/L*t_ON) + V_in/R)*exp(-R/L*t) - Ke*omega/R;

i_ON2 = @(t) 1/R*( (V_in*(exp(-R/L*t_OFF)-1) )/(1-exp(-R/L*dt)) )*exp(-R/L*t) + V_in/R - Ke*omega/R;
i_OFF2 = @(t) V_in/R*exp(-R/L*t)*( (1-exp(-R/L*t_ON))/(1-exp(-R/L*dt)) ) - Ke*omega/R;


i_mean = 1/dt * ( V_in/R*t_ON - Ke*omega/R*dt + L/R*(i0+Ke*omega/R)*(1-exp(-R/L*dt)) + L/R*V_in/R*(exp(-R/L*dt) - exp(-R/L*t_OFF)) )
i_mean2 = d*V_in/R - Ke*omega/R

i_variation_on = abs(i_ON2(t_ON) / (d*V_in/R - Ke*omega/R) - 1)
i_variation_off = abs(i_OFF2(t_OFF) / (d*V_in/R - Ke*omega/R) - 1)

t_mean_ON = -(L*log((R*(((V_in*t_ON)/R - (Ke*omega*(t_ON + t_OFF))/R - (L*(exp(-(R*(t_ON + t_OFF))/L) - 1)*(Ke*omega + R*i0))/R^2 + (L*V_in*(exp(-(R*(t_ON + t_OFF))/L) - exp(-(R*t_OFF)/L)))/R^2)/(t_ON + t_OFF) - V_in/R + (Ke*omega)/R))/(Ke*omega - V_in + R*i0)))/R;
t_mean_OFF = -(L*log((((V_in*t_ON)/R - (Ke*omega*(t_ON + t_OFF))/R - (L*(exp(-(R*(t_ON + t_OFF))/L) - 1)*(i0 + (Ke*omega)/R))/R + (L*V_in*(exp(-(R*(t_ON + t_OFF))/L) - exp(-(R*t_OFF)/L)))/R^2)/(t_ON + t_OFF) + (Ke*omega)/R)/(V_in/R + exp(-(R*t_ON)/L)*(i0 - V_in/R + (Ke*omega)/R))))/R;

s_mean_ON = t_mean_ON / dt
s_mean_OFF = (t_mean_OFF+t_ON) / dt

s_mean_ON_diff_from_mid = d/2 - s_mean_ON
s_mean_OFF_diff_from_mid = (1+d)/2 - s_mean_OFF


%%
figure(1);
t1 = 0:dt/200:t_ON;
t2 = 0:dt/200:t_OFF;
subplot(2,1,1);
plot(t1, i_ON(t1));
hold on;
plot(t_ON+t2, i_OFF(t2));
hold off;
xlim([0, dt]);
xlabel('Time [s]');
ylabel('Current [A]');
legend('ON-period', 'OFF-period');
title('PWM cycle with custom i_0 current');

subplot(2,1,2);
plot(t1, i_ON2(t1));
hold on;
plot(t_ON+t2, i_OFF2(t2));
hold off;
xlim([0, dt]);
xlabel('Time [s]');
ylabel('Current [A]');
legend('ON-period', 'OFF-period');
title('PWM cycle in current steady-state');

%%
i_mean = mean([i_ON(t1), i_OFF(t2)])
i_mean2 = 1/dt * (...
    L/R * ((i0 - (V_in-Ke*omega)/R)*exp(-R/L*t_ON) + V_in/R)*(1-exp(-R/L*t_OFF)) ...
  - Ke*omega/R*t_OFF ...
  - L/R *(i0 - (V_in-Ke*omega)/R)*exp(-R/L*t_ON) + (V_in-Ke*omega)/R*t_ON + L/R * (i0 - (V_in-Ke*omega)/R) ...
)

i_mean3 = 1/dt * (...
    V_in/R*t_ON ...
  - Ke*omega/R*dt ...
  + L/R * (i0 + Ke*omega/R) * (1-exp(-R/L*dt)) ...
  + L/R*V_in/R * (exp(-R/L*dt) - exp(-R/L*t_OFF)) ...
)

% mean below only holds if i0 is set such that start and end aligns
i_mean4 = 1/dt * (V_in/R*t_ON - Ke*omega/R*dt - 2*L/R*Ke/R*omega*exp(-R/L*dt))

%%
a = ( -L/R * (i0 - V_ON/R)*exp(-R/L*t_ON) +V_ON/R*t_ON + L/R*(i0 - V_ON/R) ) / t_ON
mean(i_ON(t1))

b = ( -L/R * ((i0 - V_ON/R)*exp(-R/L*t_ON) + V_ON/R)*(exp(-R/L*t_OFF) - 1) ) / t_OFF
mean(i_OFF(t2))

%%
i_mean3 = d*a + (1-d)*b
i_mean4 = (a*t_ON + b*t_OFF) / dt
i_mean5 = d*V_ON/R

%%
variation = max(i_ON(t1)) - min(i_ON(t1))
V_ON/R * (1 - (exp(-R/L*t_OFF) - exp(-R/L*dt))/(1-exp(-R/L*dt))) * (1-exp(-R/L*t_ON))

%%
t_ON_AVG = -L/R*log( 1/t_ON * L/R * (1-exp(-R/L*t_ON)) )
d_ON_AVG = t_ON_AVG / dt

t_OFF_AVG = -L/R*log( 1/t_OFF * L/R * (1-exp(-R/L*t_OFF)) )
d_OFF_AVG = (t_ON + t_OFF_AVG) / dt

%%
t_AVG1 = -L/R * log( t_OFF/dt * (exp(-R/L*dt)-1)/(exp(-R/L*t_OFF)-1) )
t_AVG2 = -L/R * log( t_ON/dt * (exp(-R/L*dt)-1)/(exp(-R/L*t_ON)-1) ) + t_ON

%%
i_ON_MID = V_ON/R * (exp(-R/L * t_ON/2)*(exp(-R/L*t_OFF)-1)/(1-exp(-R/L*dt)) + 1)
i_OFF_MID = V_ON/R * ( (exp(-R/L*dt) - exp(-R/L*t_ON))/(1-exp(-R/L*dt)) + 1) * exp(-R/L * t_OFF/2)

i_mean6 = (i_ON_MID+i_OFF_MID)/2