% Ramp test for mechanical parameter estimation
ramp_duty_start = 0.0;
ramp_duty_end = 1.0;
ramp_steps = 10;
ramp_step_time = 1;
ramp_time = ramp_step_time * (ramp_steps+1);

ramp_sample_times1 = (0:ramp_steps)' * ramp_step_time + ramp_step_time/2;
ramp_sample_times2 = ramp_sample_times1(end) + (0:4*ramp_steps)' * ramp_step_time/4 + ramp_step_time/8;
ramp_sample_times = [ramp_sample_times1;];% ramp_sample_times2];
ramp_sample_times = [ramp_sample_times; ramp_sample_times(end)+(0.1:0.10:5)'];
tmp = [ramp_sample_times-0.01*ramp_step_time, ramp_sample_times, ramp_sample_times+0.01*ramp_step_time];
ramp_sample_times_t = reshape(tmp', [3*size(tmp,1),1]);
ramp_sample_times_signal = repmat([0;1;0], [size(tmp,1),1]);


% dt = 0.04;
% ramp_sample_times = (0.1:dt:ramp_time)';
% tmp = [ramp_sample_times-0.1*dt, ramp_sample_times, ramp_sample_times+0.1*dt];
% ramp_sample_times_t = reshape(tmp', [3*size(tmp,1),1]);
% ramp_sample_times_signal = repmat([0;1;0], [size(tmp,1),1]);


% ramp_number_samples_each_time = 10;
% dt = 0.04;
% sample_time = 0.1;
% ramp_sample_times = [];
% num_samples = 0;
% while (sample_time < ramp_time)
%     ramp_sample_times(end+1,1) = sample_time;
%     sample_time = sample_time + dt;
%     num_samples = num_samples + 1;
%     if (num_samples == ramp_number_samples_each_time)
%         sample_time = ceil(sample_time / ramp_step_time);
%         num_samples = 0;
%     end
% end
% tmp = [ramp_sample_times-0.1*dt, ramp_sample_times, ramp_sample_times+0.1*dt];
% ramp_sample_times_t = reshape(tmp', [3*size(tmp,1),1]);
% ramp_sample_times_signal = repmat([0;1;0], [size(tmp,1),1]);
% stairs(ramp_sample_times_t, ramp_sample_times_signal)