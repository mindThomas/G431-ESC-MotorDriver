function [X_out, P_out, t_out] = MechanicalParameterEKF(X, P_prev, t_prev, ...
    R, Kt, ... % constants/parameters
    t_now, duty, Vin, coast, ... % inputs (u)
    i_avg, speed)  % measurements (z)

    % Split state vector, X[k-1], into individual variables    
    omega = X(1);
    J = X(2);
    Ke = X(3);
    B = X(4);
    tau_c = X(5);
    
    % Setup covariance matrices
    Q_proc = zeros(5,5); % process noise on constant parameter model
    Q_proc(1,1) = 0.01; % process noise on omega
    R_meas = diag([0.01, 0.03]); % measurement noise on speed and current
    
    % Preliminary input computations
    Vmot = 0;
    if (coast == false)
        Vmot = duty * Vin; % assuming average current for PWM frequency >> inductance time constants 
    end
    dt = t_now - t_prev;

    %% Prediction step  
    X_apriori = single(zeros(5,1));
     
    % Prediction step - constant parameter model
    X_apriori = X;

    % Determine model Jacobian (F)   -  OBS. This is not supposed to use/depend on apriori states!    
    F_prev = eye(5); % constant parameter model

    % Prediction of the motor angular velocity depending on the coast mode
    % Based on the mechanical motor model
    % Use motor voltage to propagate model 
    if ((abs(omega) > 10) || (abs(Vmot) > 0.001))
        if (coast == false)
            %dt = 10000;
            exp_t = exp(-1/J*(B + Kt*Ke/R)*dt);
            X_apriori(1) = omega * exp_t ...
                          + Vmot * Kt/(B*R + Kt*Ke) * (1-exp_t) ...
                          - tau_c * R/(B*R + Kt*Ke) * (1-exp_t);
            dexp_t_dJ = 1/J^2*(B + Kt*Ke/R)*dt * exp_t;
            dexp_t_dKe = -1/J*Kt/R*dt * exp_t;
            dexp_t_dB = -1/J*dt * exp_t;     

            dBRKtKe_dKe = -Kt/(B*R + Kt*Ke)^2; % d( 1/(B*R + Kt*Ke) ) / dKe
            dBRKtKe_dB = -R/(B*R + Kt*Ke)^2; % d( 1/(B*R + Kt*Ke) ) / dB

            F_prev(1,1) = exp_t; % domega / domega
            F_prev(1,2) = omega*dexp_t_dJ - Vmot*Kt/(B*R + Kt*Ke)*dexp_t_dJ + tau_c*R/(B*R + Kt*Ke)*dexp_t_dJ; % domega / dJ
            F_prev(1,3) = omega*dexp_t_dKe + Vmot*Kt*dBRKtKe_dKe*(1-exp_t) - Vmot*Kt/(B*R + Kt*Ke)*dexp_t_dKe - tau_c*R*dBRKtKe_dKe*(1-exp_t) + tau_c*R/(B*R + Kt*Ke)*dexp_t_dKe; % domega / dKe
            F_prev(1,4) = omega*dexp_t_dB + Vmot*Kt*dBRKtKe_dB*(1-exp_t) - Vmot*Kt/(B*R + Kt*Ke)*dexp_t_dB - tau_c*R*dBRKtKe_dB*(1-exp_t) + tau_c*R/(B*R + Kt*Ke)*dexp_t_dB; % domega / dB
            F_prev(1,5) = -R/(B*R + Kt*Ke) * (1-exp_t); % domega / dtau_c        
        else
            X_apriori(1) = (omega + 1/B*tau_c)*exp(-B/J*dt) - 1/B*tau_c;
            F_prev(1,1) = exp(-B/J*dt); % domega / domega
            F_prev(1,2) = B/J^2*dt * (omega + 1/B*tau_c)*exp(-B/J*dt); % domega / dJ
            F_prev(1,3) = 0; % domega / dKe
            F_prev(1,4) = (-1/B^2*tau_c)*exp(-B/J*dt) - 1/J*dt*(omega + 1/B*tau_c)*exp(-B/J*dt) + 1/B^2*tau_c; % domega / dB
            F_prev(1,5) = 1/B*exp(-B/J*dt) - 1/B; % domega / dtau_c
        end
    end   

    % Calculate apriori covariance of estimate error
    P_apriori = F_prev * P_prev * F_prev' + Q_proc;           
    
    %% Extract parameters
    omega_apriori = X_apriori(1);
    J_apriori = X_apriori(2);
    Ke_apriori = X_apriori(3);
    B_apriori = X_apriori(4);
    tau_c_apriori = X_apriori(5);

    %% Update/correction step 
    if (coast == false)
        H = zeros(2,5);

        z_hat_speed = omega_apriori;
        H(1,1) = 1; % dspeed / domega
        H(1,2) = 0; % dspeed / dJ
        H(1,3) = 0; % dspeed / dKe
        H(1,4) = 0; % dspeed / dB
        H(1,5) = 0; % dspeed / dtau_c

        % Current estimate based on steady state electro-mechanical DC motor model
        z_hat_i_avg = (Vmot - Ke_apriori*omega_apriori) / R;
        H(2,1) = -Ke_apriori/R; % di_avg / domega
        H(2,2) = 0; % di_avg / dJ
        H(2,3) = -omega_apriori/R; % di_avg / dKe
        H(2,4) = 0; % di_avg / dB
        H(2,5) = 0; % di_avg / dtau_c

        % Measurement vector    
        z = [speed;
             i_avg];  
        z_hat = [z_hat_speed;
                 z_hat_i_avg];                

        % Calculate Kalman gain
        S = H * P_apriori * H' + R_meas;        
        K = P_apriori * H' / S;  % P_apriori * H' * inv(S)        

        X_aposteriori = X_apriori + K * (z - z_hat);    
        P_aposteriori = (eye(5) - K*H) * P_apriori;     
    else % coast mode
        H = zeros(1,5);

        z_hat_speed = omega_apriori;
        H(1,1) = 1; % dspeed / domega
        H(1,2) = 0; % dspeed / dJ
        H(1,3) = 0; % dspeed / dKe
        H(1,4) = 0; % dspeed / dB
        H(1,5) = 0; % dspeed / dtau_c

        % Measurement vector    
        z = [speed];  
        z_hat = [z_hat_speed]; 

        % Calculate Kalman gain
        S = H * P_apriori * H' + R_meas(1,1);        
        K = P_apriori * H' / S;  % P_apriori * H' * inv(S)        

        X_aposteriori = X_apriori + K * (z - z_hat);    
        P_aposteriori = (eye(5) - K*H) * P_apriori;     
    end
     
    %% Send output
    X_out = X_aposteriori;
    P_out = P_aposteriori;

    t_out = t_now;
end