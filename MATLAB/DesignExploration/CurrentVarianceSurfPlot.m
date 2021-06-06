i_var = @(d,dt) (1./d) .* (1-exp(-R/L*d.*dt))./(1-exp(-R/L*dt)) - 1;

[d,freq] = meshgrid(0.1:0.05:0.9, ...
                    2000:200:20000);
dt = 1./freq;

figure;
surf(d, freq, ( i_var(d, dt)) );
xlabel('Duty Cycle [%]');
ylabel('Frequency [Hz]');