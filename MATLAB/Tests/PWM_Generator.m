function [time, PWM] = PWM_Generator(frequency, duty_cycle, n_samples)
    time = zeros(2*n_samples,1);
    PWM = zeros(2*n_samples,1);
    
    t = 0;    
    delta_t = 1/frequency;
    
    for (n = 0:(n_samples-1))
        % Start with RISING edge, hence start with HIGH
        time(2*n+1) = t;
        PWM(2*n+1) = 1;
        
        time(2*n+2) = t + duty_cycle*delta_t;
        PWM(2*n+2) = 0;    
        
        t = t + delta_t;
    end
end