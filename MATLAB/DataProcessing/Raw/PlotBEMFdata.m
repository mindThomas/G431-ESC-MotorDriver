f = fopen('C:\Users\Thomas\Desktop\bemf.txt');
data = uint8(fread(f));
fclose(f);

G = 9.143;
Voffset = 2.057;
Rsense = 0.015; % 15 mOhm

ticksInternal = 64;
nGear = 30; % gear ratio

BEMFratio = 2.2 / (2.2+10); % voltage dividers
BEMFoffset = 0.22; % from BAT30 diode

time_resolution = 1 / 100000; % 100 kHz timer
sweep_samples = 1000;
skip_samples = 2;

% Process data
% Create parsed struct
offset = 1;
[time, offset] = parseData(data, offset, 'uint32', sweep_samples);
[sample_bemf, offset] = parseData(data, offset, 'int16', sweep_samples);
[sample_encoder, offset] = parseData(data, offset, 'int32', sweep_samples);

% Change type
time = double(time(1+skip_samples:end)) * time_resolution;
sample_bemf = double(sample_bemf(1+skip_samples:end));
sample_encoder = double(sample_encoder(1+skip_samples:end));

clear data;
data.time = time - time(1);
data.encoder = sample_encoder - sample_encoder(1);
data.angle_inner = 2*pi * data.encoder / ticksInternal;
data.angle_outer = 2*pi * data.encoder / (ticksInternal*nGear);
data.bemf_raw = sample_bemf;
data.bemf_adc_volt = (data.bemf_raw / 4095) * 3.3;

ratio_shift_idx = find(diff(data.bemf_raw) > 1000);
bemf_ratio = ones(length(data.time), 1);
data.bemf = data.bemf_adc_volt;
%vadc = (in - offset) * BEMFratio + offset;
%vadc = in * BEMFratio + offset * (1 - BEMFratio);
%in = (vadc - offset * (1 - BEMFratio)) / BEMFratio
data.bemf(1:ratio_shift_idx) = (data.bemf_adc_volt(1:ratio_shift_idx) - BEMFoffset * (1 - BEMFratio)) / BEMFratio;

angular_velocity = diff(data.angle_outer) ./ diff(data.time);
angular_velocity_time = (data.time(1:end-1) + data.time(2:end)) / 2;

data.angular_velocity = interp1(angular_velocity_time, angular_velocity, data.time);

%
figure(1);
ax1 = subplot(2,1,1);
yyaxis left;
plot(data.time, data.bemf);
ylabel('BEMF voltage [V]');

yyaxis right;
plot(data.time, abs(data.angular_velocity));
ylabel('Angular velocity [rad/s]');

ax2 = subplot(2,1,2);
plot(data.time, data.bemf ./ abs(data.angular_velocity));
linkaxes([ax1, ax2], 'x');
ylabel('Ke [V / rad/s]')

xlabel('Time [s]');
xlim([0, 1.2]);

%%
idx0 = 3;
idx1 = find(data.time > 1.5);
idx1 = idx1(1);

coeffs = [abs(data.angular_velocity(idx0)), 1, 0];

model = @(coeff,t)(coeff(3) + coeff(1) * exp(-coeff(2)*t(:, 1)));
coeffs = fitNonlinearModel(data.time(idx0:idx1), abs(data.angular_velocity(idx0:idx1)), model, coeffs)
angular_velocity_fitted = model(coeffs, data.time(idx0:idx1));

figure(3);
plot(data.time(idx0:idx1), abs(data.angular_velocity(idx0:idx1)));
hold on;
plot(data.time(idx0:idx1), angular_velocity_fitted);
hold off;

ylabel('Angular velocity [rad/s]')
xlabel('Time [s]');
legend('Measured', 'Fitted');

% B/J fraction
%BJ_fraction = dmodel / model
BJ_fraction = coeffs(2)
tau_c_B_fraction = -coeffs(3)
omega0 = coeffs(1) + coeffs(3)