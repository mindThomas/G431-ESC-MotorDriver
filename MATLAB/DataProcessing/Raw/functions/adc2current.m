function current = adc2current(adc_value)
    G = 9.143;
    Voffset = 2.057; % volt
    Rsense = 0.015; % 15 mOhm
    Vref = 3.3; % volt
    ADCrange = 2^12; % 12-bit
    
    sense_voltage = (adc_value / (ADCrange-1)) * Vref;
    current = (sense_voltage - Voffset) / (G * Rsense);
end