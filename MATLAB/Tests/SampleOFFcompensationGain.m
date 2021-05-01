function G_compensation = SampleOFFcompensationGain(f, duty, R, L)
    dt = 1/f;
    t_ON = duty*dt;
    t_OFF = dt - t_ON;

    % Gain compensation comes from = i_mean / i_OFF_MID
    G_compensation = duty ./ (( (exp(-R/L*dt) - exp(-R/L.*t_ON))/(1-exp(-R/L*dt)) + 1) .* exp(-R/L .* t_OFF/2));