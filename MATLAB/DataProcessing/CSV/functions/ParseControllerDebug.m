function out = ParseControllerDebug(array)   
    if (size(array,2) ~= 9) 
        error('Incorrect dump array size when parsing');
    end
    out.time = array(:,1);
    
  	out.current_setpoint = array(:,2);
	out.current_measurement = array(:,3);
	out.current_filtered = array(:,4);

	out.omega_measurement = array(:,5);
	out.omega_filtered = array(:,6);

	out.integral = array(:,7);
	out.PI = array(:,8);
	out.duty = array(:,9);    
end