% Chirp test
chirp_duty_cycle = 0.5;
chirp_freq_start = 200;
chirp_freq_end = 1000;
chirp_freq_steps = 1000;
chirp_total_cycles = 1000;

[PWM, SampleON, SampleOFF] = PWMChirpTimeseries(chirp_duty_cycle, chirp_freq_start, chirp_freq_end, chirp_total_cycles, chirp_freq_steps);

% Override parameters from Simulation
PWM_Frequency = chirp_freq_end; % Hz
PWM_Resolution = 100; % steps

%%
chirp_n_samples = round(chirp_total_cycles / chirp_freq_steps);
chirp_total_cycles = chirp_n_samples*chirp_freq_steps;
chirp_f_delta = (chirp_freq_end - chirp_freq_start) / (chirp_freq_steps-1);
chirp_freqs = chirp_f_delta * (0:(chirp_freq_steps-1)) + chirp_freq_start;
chirp_delta_t = 1 ./ chirp_freqs;
chirp_t_end = chirp_n_samples * sum(chirp_delta_t);
