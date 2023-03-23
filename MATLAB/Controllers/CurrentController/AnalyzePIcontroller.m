scriptDir = fileparts(mfilename('fullpath'));
addpath(fullfile(scriptDir, '../../DataProcessing/CSV'));

dump = LoadControllerDump('C:\Users\Thomas\Documents\motordriver-python', '2020-07-20_00-11-32-305_controller.csv')
dump = Trim(dump, 1);

%%
%plot(dump.time, dump.duty);
plotFFT(dump.current_raw, 1/mean(diff(dump.time)));

%%
figure(10);
plot(dump.time, dump.current_raw);
hold on;
plot(dump.time, dump.current_filtered);
plot(dump.time, dump.current_setpoint);
hold off;

%%
figure(10);
plot(dump.time, dump.omega_raw);
hold on;
plot(dump.time, dump.omega_filtered);
hold off;