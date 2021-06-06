function adc_value = current2adc(current)
    G = 9.143;
    Voffset = 2.057; % volt
    Rsense = 0.015; % 15 mOhm
    Vref = 3.3; % volt
    ADCrange = 2^12; % 12-bit
        
    Vcurrent = (G * Rsense) * current;
    sense_voltage = Vcurrent + Voffset;
    adc_value = (ADCrange-1) * (sense_voltage / Vref);
end