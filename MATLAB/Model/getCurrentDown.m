function iOFF = getCurrentDown(R, L, Ke, omega, imax, t)
    iOFF = (imax + Ke*omega/R) * exp(-R/L * t) - Ke*omega/R;
end