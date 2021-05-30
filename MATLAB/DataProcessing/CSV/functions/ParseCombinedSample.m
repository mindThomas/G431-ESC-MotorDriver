function out = ParseCombinedSample(array)   
    if (size(array,2) ~= 12) 
        error('Incorrect dump array size when parsing');
    end
    out.time = array(:,1);
    
  	out.PWM_Frequency = array(:,2);
	out.TimerMax = array(:,3);
	out.DutyCycleLocation = array(:,4);
	out.TriggerLocationON = array(:,5);
	out.TriggerLocationOFF = array(:,6);

	out.CurrentON = array(:,7);
	out.CurrentOFF = array(:,8);
	
    out.Bemf = array(:,9);   
    
    out.VbusON = array(:,10);
    out.VbusOFF = array(:,11);
    out.Encoder = array(:,12);
end