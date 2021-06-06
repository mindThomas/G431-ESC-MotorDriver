function iON = getCurrentUp(R, L, Ke, Vin, omega, i0, t)
    iON = (i0 - (Vin - Ke*omega)/R) * exp(-R/L * t) + (Vin - Ke*omega)/R;
end