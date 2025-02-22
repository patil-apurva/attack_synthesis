function S_tau_all = simulateMC_ut(eps_t_all_1, eps_t_all_2, xt_prime_1, xt_prime_2, xt_prime_3, xt_prime_4, f_xt_prime_1, f_xt_prime_2, f_xt_prime_3, f_xt_prime_4, t, h, T, b, s, danger_y1, danger_y2, eta, k1, k2, k3)

    eps_t_prime_1 = eps_t_all_1; %standard normal noise at t
    eps_t_prime_2 = eps_t_all_2; %standard normal noise at t
    
    S_tau = 0; %the cost-to-go of the state dependent cost of a sample path;
    
    for t_prime = t:h:T % this loop is to compute S(tau_i)
                
        S_tau = S_tau + h*b*(xt_prime_1*xt_prime_1 + xt_prime_2*xt_prime_2); %add the state dependent running cost
        
        xt_prime_1 = xt_prime_1 + f_xt_prime_1*h; %move tau ahead
        xt_prime_2 = xt_prime_2 + f_xt_prime_2*h; %move tau ahead
        xt_prime_3 = xt_prime_3 + f_xt_prime_3*h + s*eps_t_prime_1*sqrt(h); %move tau ahead
        xt_prime_4 = xt_prime_4 + f_xt_prime_4*h + s*eps_t_prime_2*sqrt(h); %move tau ahead

        if ((xt_prime_2>=danger_y1) && (xt_prime_2<=danger_y2) && (xt_prime_1>=-0.07) && (xt_prime_1<=0.07))%if yes means t_prime=t_exit
            S_tau = S_tau + eta; %add the boundary cost to S_tau
        end

        eps_t_prime_1 = randn; %standard normal noise at new t_prime. Will be used in the next iteration 
        eps_t_prime_2 = randn; %standard normal noise at new t_prime. Will be used in the next iteration 
        
        f_xt_prime_1 =  k1*xt_prime_1 + xt_prime_3*cos(xt_prime_4); %f_xt_prime_1 at new t_prime. Will be used in the next iteration 
        f_xt_prime_2 =  k1*xt_prime_2 + xt_prime_3*sin(xt_prime_4); %f_xt_prime_2 at new t_prime. Will be used in the next iteration 
        f_xt_prime_3 =  k2*xt_prime_3; %f_xt_prime_3 at new t_prime. Will be used in the next iteration 
        f_xt_prime_4 =  k3*xt_prime_4; %f_xt_prime_4 at new t_prime. Will be used in the next iteration  
    end
    
    S_tau_all = S_tau;