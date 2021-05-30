scriptDir = fileparts(mfilename('fullpath'));
addpath(fullfile(scriptDir, 'functions'));
addpath(fullfile(scriptDir, '../../Parameters'));
Constants_ESC;

DumpFolder = '../../../Python/';
data = LoadDump(DumpFolder, '2021-05-25_01-40-44-837_controller.csv');

%close all;

fig = figure(1); set(fig, 'NumberTitle', 'off', 'Name', 'Current Controller');
ax1 = subplot(3,1,1); plot(data.time, data.current_measurement, data.time, data.current_filtered, data.time, data.current_setpoint); legend('Measured', 'Filtered', 'Setpoint'); title('Current'); ylabel('A');
ax2 = subplot(3,1,2); plot(data.time, data.omega_measurement, data.time, data.omega_filtered); legend('Measured', 'Filtered'); title('Angular velocity'); ylabel('rad/s');
ax3 = subplot(3,1,3); plot(data.time, data.duty, data.time, data.PI_out, data.time, data.PI_out-data.integral); legend('Duty cycle', 'PI', 'P', 'Integral'); title('Controller');
linkaxes([ax1,ax2,ax3],'x');
xlabel('Time [s]');
