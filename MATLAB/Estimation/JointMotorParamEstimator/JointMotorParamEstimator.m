classdef JointMotorParamEstimator   
    properties
        X
        P
    end
    properties (Access = private)                 

    end    
    
    methods
        function obj = JointMotorParamEstimator(R_init, Ke_init)            
            obj.X = [R_init; Ke_init; 0; 0];
            obj.P = diag([0.5, 0.1, 0, 0]);
        end   
        
        function obj = Predict(obj, i, omega, sigma2_i, sigma2_omega)      
            % Only perform prediction step when there is a jump in Vmot
            X_apriori = obj.X;            
            X_apriori(1) = obj.X(1); % R
            X_apriori(2) = obj.X(2); % Ke                       
            % Force change the states of the changed inputs to be smoothed
            X_apriori(3) = i; % i   
            X_apriori(4) = omega; % omega                                  
            
            Q_change = zeros(4);
            Q_change(1,1) = 0;
            Q_change(2,2) = 0;
            Q_change(3,3) = sigma2_i;
            Q_change(4,4) = sigma2_omega;            
            
            % Calculate apriori covariance of estimate error
            obj.P = obj.P + Q_change;     
            
            % Set apriori output
            obj.X = X_apriori;
        end
        
        function obj = Update(obj, i, omega, Vmot, sigma2_i, sigma2_omega, sigma2_Vmot)
            % Compute measurement estimate                        
            z_i = i;
            z_hat_i = obj.X(3);
            
            z_omega = omega;
            z_hat_omega = obj.X(4);
            
            % Vmot = R*i + Ke*omega
            z_Vmot = Vmot;
            z_hat_Vmot = obj.X(1)*obj.X(3) + obj.X(2)*obj.X(4);            
            
            % Form measurement vector and measurement estimate vector
            z = [z_i; z_omega; z_Vmot];
            z_hat = [z_hat_i; z_hat_omega; z_hat_Vmot];
            
            % Compute Jacobian
            H = zeros(3, 4);
            H(1,3) = 1;
            H(2,4) = 1;
            H(3,1) = obj.X(3);
            H(3,2) = obj.X(4);
            H(3,3) = obj.X(1);
            H(3,4) = obj.X(2);
            
            R = diag([sigma2_i, sigma2_omega, sigma2_Vmot]);
            
            % Calculate Kalman gain
            S = H * obj.P * H' + R;        
            K = obj.P * H' / S;  % P_apriori * H' * inv(S)        
 
            % Compute aposteriori
            obj.X = obj.X + K * (z - z_hat);    
            obj.P = (eye(4) - K*H) * obj.P;  
        end
        
        
        
        function obj = Predict2(obj, Vmot, omega, sigma2_Vmot, sigma2_omega)            
            X_apriori = obj.X;
            
            X_apriori(1) = obj.X(1); % R
            X_apriori(2) = obj.X(2); % Ke            
            % i[k] = (Vmot[k] + Ke[k]*omega[k]) / R[k]
            % i[k] = (Vmot[k-1]+delta_Vmot[k] + Ke[k]*(omega[k-1]+delta_omega[k])) / R[k]
            % Assuming aposteriori R[k-1|k-1] and Ke[k-1|k-1] to be the same as
            % apriori R[k|k-1] and Ke[k|k-1]
            % i[k] = i[k-1] + (delta_Vmot[k] + Ke[k]*(delta_omega[k])) / R[k]
            delta_Vmot = Vmot - obj.prev_Vmot;
            delta_omega = omega - obj.prev_omega;
            X_apriori(3) = obj.X(3) + (delta_Vmot - obj.X(2)*delta_omega) / obj.X(1); % i   
            
            % Determine model Jacobian (F)   -  OBS. This is not supposed to use/depend on apriori states!
            % Constant state model
            F_prev = eye(3);   
            F_prev(3,1) = -(delta_Vmot - obj.X(2)*delta_omega) / obj.X(1)^2;
            F_prev(3,2) = delta_omega / obj.X(1);
            F_prev(3,3) = 1;
                
            % u = [delta_Vmot; delta_omega]
            Q_u = diag([2*sigma2_Vmot, 2*sigma2_omega]); % times 2 because of delta
            % dX_apriori / du
            dX_apriori_du = zeros(3);
            dX_apriori_du(3,1) = 1 / obj.X(1);
            dX_apriori_du(3,2) = -obj.X(2) / obj.X(1);            
            Q_extra = dX_apriori_du * Q_u;
            
            % Calculate apriori covariance of estimate error
            obj.P = F_prev * obj.P * F_prev' + obj.Q + Q_extra;     
            
            % Set apriori output
            obj.X = X_apriori;
        end        
    end
end