function G_compensation = SampleONcompensationGain(f, duty, R, L)
    dt = 1/f;
    t_ON = duty*dt;
    t_OFF = dt - t_ON;

    % Gain compensation comes from = i_mean / i_ON_MID
    G_compensation = duty ./ (exp(-R/L .* t_ON/2).*(exp(-R/L.*t_OFF)-1)/(1-exp(-R/L*dt)) + 1);