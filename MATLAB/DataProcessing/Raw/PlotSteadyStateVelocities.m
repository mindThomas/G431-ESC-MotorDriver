f = fopen('C:\Users\Thomas\Desktop\steady_state_velocities2.txt');
data = uint8(fread(f));
fclose(f);

G = 9.143;
Voffset = 2.0545;
Rsense = 0.015; % 15 mOhm

ticksInternal = 64;
nGear = 30; % gear ratio

Kt = 0.157 / nGear;

time_resolution = 1 / 100000; % 100 kHz timer
sweep_samples = 2400;
skip_samples = 0;

% Process data
% Create parsed struct
offset = 1;
[time, offset] = parseData(data, offset, 'uint32', sweep_samples);
[sample_current_sense, offset] = parseData(data, offset, 'int16', sweep_samples);
[sample_encoder, offset] = parseData(data, offset, 'int32', sweep_samples);

% Change type
time = double(time(1+skip_samples:end)) * time_resolution;
sample_current_sense = double(sample_current_sense(1+skip_samples:end));
sample_encoder = double(sample_encoder(1+skip_samples:end));

[time,idx] = sortrows(time);
sample_current_sense = sample_current_sense(idx);
sample_encoder = sample_encoder(idx);

clear data;
data.time = time - time(1);
data.raw = sample_current_sense;
data.encoder = sample_encoder - sample_encoder(1);
data.angle_inner = 2*pi * data.encoder / ticksInternal;
data.angle_outer = 2*pi * data.encoder / (ticksInternal*nGear);
data.sense_voltage = (data.raw / 4095) * 3.3;

idx = @(step) (step*100+1):((step+1)*100);
Voffset = mean(data.sense_voltage(idx(0)));

data.current = (data.sense_voltage - Voffset) / (G * Rsense);

angular_velocity = diff(data.angle_inner) ./ diff(data.time);
angular_velocity_time = (data.time(1:end-1) + data.time(2:end)) / 2;
data.angular_velocity = interp1(angular_velocity_time, angular_velocity, data.time);


%%
step = 23;

figure(1);
ax1 = subplot(2,1,1);
plot(data.time(idx(step)), data.current(idx(step)));

ax2 = subplot(2,1,2);
plot(data.time(idx(step)), data.angular_velocity(idx(step)));

linkaxes([ax1, ax2], 'x');

%%
for (step = 1:23)    
    i = idx(step);
    steady_state_current(step,1) = mean(data.current(i));
    steady_state_angular_velocity(step,1) = abs((data.angle_inner(i(end)) - data.angle_inner(i(1))) / (data.time(i(end)) - data.time(i(1))));
end

figure(2);
plot(steady_state_angular_velocity, steady_state_current, '.r');

l = Line(false);
l = l.fit(steady_state_angular_velocity, steady_state_current)
l.plot(min(steady_state_angular_velocity), max(steady_state_angular_velocity), 'b');

xlabel('Angular velocity (inner = before gearing) [rad/s]');
ylabel('Steady state current [A]');

%%
B = Kt * l.a
tau_c = Kt * l.b

%% Enforce constraint from PlotBEMFdata.m
%tau_c/B = 495.724
%tau_c = 495.724 * B
%B = tau_c / 495.724

%l.a = B / Kt
%l.b = tau_c / Kt
%y = l.a * x + l.b
%y = l.a * x + 495.724*l.a
%y = l.a * (x + 495.724)

l = Line(true);
l = l.fit(steady_state_angular_velocity+495.724, steady_state_current)



l.b = 495.724*l.a;
l.plot(min(steady_state_angular_velocity), max(steady_state_angular_velocity), 'g');

B = Kt * l.a
tau_c = Kt * l.b

enforced_ratio = tau_c/B