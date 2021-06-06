function volt = adc2volt(adc_value)    
    Vref = 3.3; % volt
    ADCrange = 2^12; % 12-bit
        
    volt = (adc_value / (ADCrange-1)) * Vref;
end