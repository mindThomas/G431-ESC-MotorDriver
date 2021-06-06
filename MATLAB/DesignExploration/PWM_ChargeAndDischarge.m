f = 2000; % PWM_Frequency
d = 0.5; % PWM_Duty
dt = 1/f;
t_ON = d*dt;
t_OFF = (1-d)*dt;
V_ON = 9;

i0 = V_ON/R * (exp(-R/L*t_OFF) - exp(-R/L*dt)) / (1-exp(-R/L*dt));

i_ON = @(t) (i0-V_ON/R)*exp(-R/L*t) + V_ON/R;
i_OFF = @(t) i_ON(t_ON)*exp(-R/L*t);

figure(1);
t1 = 0:dt/200:t_ON;
t2 = 0:dt/200:t_OFF;
plot(t1, i_ON(t1));
hold on;
plot(t_ON+t2, i_OFF(t2));
hold off;
xlim([0, dt]);
xlabel('Time [s]');
ylabel('Current [A]');
legend('ON-period', 'OFF-period');


i_mean = mean([i_ON(t1), i_OFF(t2)])
i_mean2 = ( V_ON/R*t_ON + L/R*i0*(1-exp(-R/L*dt)) + L/R*V_ON/R*exp(-R/L*dt) - L/R*V_ON/R*exp(-R/L*t_OFF) ) / dt

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