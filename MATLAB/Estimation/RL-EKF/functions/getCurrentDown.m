function i1 = getCurrentDown(R, L, i0, t)
    i1 = i0 * exp(-R/L * t);
end