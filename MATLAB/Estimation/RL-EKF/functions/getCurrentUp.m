function i1 = getCurrentUp(R, L, Vbat, i0, t)
    i1 = Vbat/R * (1 - (1-i0*R/Vbat) * exp(-R/L * t));    
end