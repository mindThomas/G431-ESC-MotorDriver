function out = ParseCombinedSample(array, varargin)
    if (length(varargin) > 0)
        filter_speed = varargin{1};
    end
    if (size(array,2) ~= 12) 
        error('Incorrect dump array size when parsing');
    end
    out.time = array(:,1);
    
    out.PWM_Frequency = array(:,2);
    out.TimerMax = array(:,3);
    
    out.DutyCycleLocation = array(:,4);
    out.DutyCycle = out.DutyCycleLocation ./ out.TimerMax;
    out.TriggerLocationON = array(:,5);
    out.TriggerLocationOFF = array(:,6);
    out.TriggerON = out.TriggerLocationON ./ out.TimerMax;
    out.TriggerOFF = out.TriggerLocationOFF ./ out.TimerMax;
        
    out.CurrentON = array(:,7);
    out.CurrentOFF = array(:,8); 
    out.Bemf = array(:,9); 
    
    out.VinON = array(:,10);
    out.VinOFF = array(:,11);
    
    out.Vin = (out.VinON + out.VinOFF) ./ ...
              (sum( [out.VinON ~= 0, out.VinOFF ~= 0], 2));
     
    out.Encoder = array(:, 12);    
    

    TicksPrRev = 1920; % this should be moved out of here and loaded from a config
    if (filter_speed)
        t_speed_downsampled = out.time(1):0.005:out.time(end);
        encoder_downsampled = interp1(out.time, out.encoder, t_speed_downsampled);        
        speed = 2*pi/TicksPrRev * diff(encoder_downsampled) ./ diff(t_speed_downsampled);
        speed(end+1) = speed(end);
    
        time_speed = (t_speed_downsampled(1:end-1) + t_speed_downsampled(2:end)) / 2;
        time_speed(end+1) = 2*time_speed(end) - time_speed(end-1);                
    else             
        speed = 2*pi/TicksPrRev * diff(out.encoder) ./ diff(out.time);
        speed(end+1) = speed(end);
    
        time_speed = (out.time(1:end-1) + out.time(2:end)) / 2;
        time_speed(end+1) = 2*time_speed(end) - time_speed(end-1);                
    end
    out.Speed = interp1(time_speed, speed, out.time, 'linear', 'extrap');   
    out.RPM = 60 * out.speed / (2*pi);    
end