scriptDir = fileparts(mfilename('fullpath'));
addpath(fullfile(scriptDir, 'functions'));
addpath(fullfile(scriptDir, '../../Parameters'));
Constants_ESC;

DumpFolder = '../../../Python/';
data = LoadDump(DumpFolder, '2021-05-30_22-46-11-674_raw.csv');
data.dutyCycle = data.DutyCycleLocation ./ data.TimerMax;
data.triggerLocation = data.TriggerLocationOFF ./ data.TimerMax;

%close all;

fig = figure(1); set(fig, 'NumberTitle', 'off', 'Name', 'Current measurement');
ax1 = subplot(2,1,1); plot(data.TimerFrequency); legend('PWM Frequency');
%ax2 = subplot(3,1,2); plot(data.triggerLocation); legend('Trigger location');
ax3 = subplot(2,1,2); plot(data.CurrentOFF); legend('Current'); ylabel('Current [A]');
%ax2 = subplot(3,1,2); plot(data.time, data.omega_measurement, data.time, data.omega_filtered); legend('Measured', 'Filtered'); title('Angular velocity'); ylabel('rad/s');
%ax3 = subplot(3,1,3); plot(data.time, data.duty, data.time, data.PI_out, data.time, data.PI_out-data.integral); legend('Duty cycle', 'PI', 'P', 'Integral'); title('Controller');
linkaxes([ax1,ax3],'x');
xlabel('Samples');

dcm_obj = datacursormode(gcf);
set(dcm_obj, 'UpdateFcn', @(~,evt) currentTooltip(evt,ax3,data))

function txt = currentTooltip(event_obj, target_axis, data)
    pos = get(event_obj,'Position');
    idx = get(event_obj,'DataIndex');
        
    % check if looking at provided axis
    if (target_axis == event_obj.Target) || ...
        (isobject(target_axis) ...
            && ismember('Children',properties(target_axis)) ...
            && target_axis.Children == event_obj.Target)

        txt = {   ['Index: ',num2str(idx)],...
                  ['Current: ',num2str(pos(2))],...
                  ['Sampling location: ',num2str(data.TriggerLocationOFF(idx) / data.TimerMax(idx))]...
                  ['Time within period: ',num2str(1000/data.TimerFrequency(idx) * data.TriggerLocationOFF(idx) / data.TimerMax(idx)),' ms']...
                  };     
    else
        % else output default text
      txt = {   ['X: ',num2str(pos(1))],...
          ['Y: ',num2str(pos(2))]...
          };   
    end
end