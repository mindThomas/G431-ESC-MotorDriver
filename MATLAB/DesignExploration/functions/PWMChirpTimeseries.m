function [PWM, SampleON, SampleOFF] = PWMChirpTimeseries(duty_cycle, f_start, f_end, total_cycles, steps)
    n_samples = round(total_cycles / steps);
    total_cycles = n_samples*steps;

    PWM_time = zeros(2*total_cycles,1);
    PWM = zeros(2*total_cycles,1);
    SampleON_time = zeros(2*total_cycles,1);
    SampleON = zeros(2*total_cycles,1);
    SampleOFF_time = zeros(2*total_cycles,1);
    SampleOFF = zeros(2*total_cycles,1);    
    
    f_delta = (f_end - f_start) / (steps-1);
    t0 = 0;
    for (i = 0:(steps-1))
        f = f_start + i*f_delta;
        dt = 1/f;
        
        [t,s] = PWM_Generator(f, duty_cycle, n_samples);
        PWM_time(2*n_samples*i+1:2*n_samples*(i+1)) = t + t0;
        PWM(2*n_samples*i+1:2*n_samples*(i+1)) = s;
        
        [t,s] = PWM_Generator(f, duty_cycle / 2, n_samples);
        SampleON_time(2*n_samples*i+1:2*n_samples*(i+1)) = t + t0;
        SampleON(2*n_samples*i+1:2*n_samples*(i+1)) = s;        
        
        [t,s] = PWM_Generator(f, 1-0.5*(1-duty_cycle), n_samples);
        SampleOFF_time(2*n_samples*i+1:2*n_samples*(i+1)) = t + t0;
        SampleOFF(2*n_samples*i+1:2*n_samples*(i+1)) = s;                
        
        t0 = t0 + t(end-1) + dt;
    end

    PWM = timeseries(boolean(PWM), PWM_time);    
    SampleON = timeseries(boolean(SampleON), SampleON_time);
    SampleOFF = timeseries(boolean(SampleOFF), SampleOFF_time);
end