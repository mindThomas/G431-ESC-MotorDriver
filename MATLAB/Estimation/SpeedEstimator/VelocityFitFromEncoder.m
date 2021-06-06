function speed_fitted = VelocityFitFromEncoder(time, speed, t0, t1, time_out)
    [~, idx0] = min(abs(time - t0));
    [~, idx1] = min(abs(time - t1));

    coeffs = [abs(speed(idx0)), 1, 0];

    model = @(coeff,t)(coeff(3) + coeff(1) * exp(-coeff(2)*t(:, 1)));
    coeffs = fitNonlinearModel(time(idx0:idx1), abs(speed(idx0:idx1)), model, coeffs)    
    
    [a,b,c] = fitExponentialDecay2(time(idx0:idx1), abs(speed(idx0:idx1)))
    coeffs = [b, c, a];
    
    speed_fitted = model(coeffs, time_out);
    return
    
    x = time(idx0:idx1);
    y = abs(speed(idx0:idx1));
    
    S = [0];
    for (k = 2:length(x))
        S(k,1) = S(k-1,1) + 1/2*(y(k)+y(k-1))*(x(k)-x(k-1));
    end
    
    A = zeros(2,2);
    A(1,1) = sum( (x-x(1)) .* (x-x(1)) );
    A(1,2) = sum( (x-x(1)) .* S );
    A(2,1) = A(1,2);
    A(2,2) = sum( S .* S );
    
    B = zeros(2,1);
    B(1) = sum( (y-y(1)) .* (x-x(1)) );
    B(2) = sum( (y-y(1)) .* S );
    
    res = inv(A) * B;
    c = res(2)
    
    A(1,1) = length(x);
    A(1,2) = sum( exp(c*x) );
    A(2,1) = A(1,2);
    A(2,2) = sum( exp(2*c*x) );
    
    B(1) = sum(y);
    B(2) = sum( y .* exp(c*x) );
    
    res = inv(A) * B;
    a = res(1)
    b = res(2)
    
    coeffs = [b, c, a];
    speed_fitted = model(coeffs, time_out);
    
end