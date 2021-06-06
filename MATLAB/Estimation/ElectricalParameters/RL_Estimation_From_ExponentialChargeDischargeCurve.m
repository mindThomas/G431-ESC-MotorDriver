data = LoadDump('C:\Users\Thomas\Documents\motordriver-python\', '2020-05-10_22-43-14-484.csv');
idx = find(data.duty_cycle > 0);
data = Extract(data, idx(1), idx(end));

data.current = zeros(length(data.time),1);
data.current(data.current_on ~= 0) = data.current_on(data.current_on ~= 0);
data.current(data.current_off ~= 0) = data.current_off(data.current_off ~= 0);

data.trigger = zeros(length(data.time),1);
data.trigger(data.current_on ~= 0) = data.trigger_on(data.current_on ~= 0);
data.trigger(data.current_off ~= 0) = data.trigger_off(data.current_off ~= 0);

idx_change = find(diff(data.timer_frequency));

%% Charge
data_charge = Extract(data, 2, idx_change(1)/2);
dt = 1/mean(data_charge.timer_frequency);

figure(1);
subplot(2,1,1);
plot(data_charge.trigger*dt, data_charge.current);
[a,b,c] = fitExponentialDecay2(data_charge.trigger*dt, data_charge.current);
hold on;
fplot(@(x) a + b .* exp(c.*x), [min(data_charge.trigger*dt), max(data_charge.trigger*dt)]);
hold off;

%% Discharge
data_discharge = Extract(data, idx_change(1)/2, idx_change(1));
dt = 1/mean(data_discharge.timer_frequency);

figure(1);
subplot(2,1,2);
plot(data_discharge.trigger*dt, data_discharge.current);
[a,b,c] = fitExponentialDecay2(data_discharge.trigger*dt, data_discharge.current);
hold on;
fplot(@(x) a + b .* exp(c.*x), [min(data_discharge.trigger*dt), max(data_discharge.trigger*dt)]);
hold off;

%%
% a = current during ON after transient response
% c = -R/L
R_est = data.vin / a
L_est = -R_est / c