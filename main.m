clearvars
close all
run('parameters.m')

seed = 1234;
rng( seed ); % Reset the CPU random number generator.
gpurng( seed ); % Reset the GPU random number generator.

fail_cnt = 0; %number of trajectories failed
    
for traj_itr = 1:traj_num
        traj_itr

    X = []; %to store all positions of this trajectory
    X = [X, x0]; %stack the initial position

    xt = x0; %start the state from the given initial position
    % dynamics model is unicycle model
    f_xt = [k1*xt(1) + xt(3)*cos(xt(4)); k1*xt(2) + xt(3)*sin(xt(4)); k2*xt(3); k3*xt(4)]; %initial f_xt
    safe_flag_traj = 1;

    is_fail_traj_counted = 0;

    for t = t0:h:T-h % this loop is to find u(t), theta(t), and x(t) at each time step t => x(t+h) = x(t) + f(x(t)).h + G_u.u(t).h + Sigma*(theta(t).h + dw)
        % Note that process noises are only applied to the last two states,
        % which are directly actuated
        eps_t_all_1_ut = randn(1, runs, 'gpuArray'); %GPU array that stores eps_1(t) at the start of each sample path starting at time t and state xt
        eps_t_all_2_ut = randn(1, runs, 'gpuArray'); %GPU array that stores eps_2(t) at the start of each sample path starting at time t and state xt

        S_tau_all_ut = arrayfun(@simulateMC_ut, eps_t_all_1_ut, eps_t_all_2_ut, xt(1), xt(2), xt(3), xt(4), ...
            f_xt(1), f_xt(2), f_xt(3), f_xt(4), t, h, T, b, s, danger_y1, danger_y2,...
            eta, k1, k2, k3); %an array that stores S(tau) of each sample path starting at time t and state xt

        eps_t_all_arr_ut = gather([eps_t_all_1_ut; eps_t_all_2_ut]); %concatenate eps_t_all_arr_1 and eps_t_all_arr_2 in an array

        denom_i_ut = exp(-S_tau_all_ut/alpha); %(size: (1 X runs))
        numer_ut = eps_t_all_arr_ut*(denom_i_ut.'); %(size: (2 X 1))
        denom_ut = sum(denom_i_ut); %scalar

        ut = (s/sqrt(h)) * (numer_ut/denom_ut); %the agent control input

        % if(any(isnan(ut(:))))
        %     fprintf("ut error!")
        %     return
        % end
        %==================================================================
        eps_t_all_1_theta_t = randn(1, runs, 'gpuArray'); %GPU array that stores eps_1(t) at the start of each sample path starting at time t and state xt
        eps_t_all_2_theta_t = randn(1, runs, 'gpuArray'); %GPU array that stores eps_2(t) at the start of each sample path starting at time t and state xt

        S_tau_all_theta_t = arrayfun(@simulateMC_theta_t, eps_t_all_1_theta_t, eps_t_all_2_theta_t, xt(1), xt(2), xt(3), xt(4), f_xt(1), f_xt(2), f_xt(3), f_xt(4), ut(1), ut(2),...
            t, h, T, a, b, s, danger_y1, danger_y2, eta, k1, k2, k3); %an array that stores S(tau) of each sample path starting at time t and state xt

        eps_t_all_arr_theta_t = gather([eps_t_all_1_theta_t; eps_t_all_2_theta_t]); %concatenate eps_t_all_arr_1 and eps_t_all_arr_2 in an array

        denom_i_theta_t = exp(-S_tau_all_theta_t/lambda); %(size: (1 X runs))
        numer_theta_t = eps_t_all_arr_theta_t*(denom_i_theta_t.'); %(size: (2 X 1))
        denom_theta_t = sum(denom_i_theta_t); %scalar

        theta_t = (1/sqrt(h)) * (numer_theta_t/denom_theta_t); %the agent control input
        % theta_t = zeros(2,1);

        % if(any(isnan(theta_t(:))))
        %     fprintf("theta_t error!")
        %     return
        % end
        
        %move the trajectory forward
        eps = randn(2,1);
        %update the position with the control inputs ut and theta_t => x(t+h) = x(t) + f(x(t)).h + G_u.u(t).h + Sigma*(theta(t).h + dw)
        xt = xt + f_xt*h + G_u*(ut*h + s*theta_t*h + s*eps*sqrt(h));
%             xt = xt + f_xt*h + G_u*(ut*h);

        X = [X, xt]; %stack the new position

        if((xt(2)>=danger_y1) && (xt(2)<=danger_y2) && (xt(1)>=-0.07) && (xt(1)<=0.07) && is_fail_traj_counted == 0) %if yes means trajectory has crossed the safe set

            fail_cnt = fail_cnt+1; 
            is_fail_traj_counted = 1;
            safe_flag_traj = 0;
            % break;
        end 

        f_xt = [k1*xt(1) + xt(3)*cos(xt(4)); k1*xt(2) + xt(3)*sin(xt(4)); k2*xt(3); k3*xt(4)]; %update f(x(t)) for the next t => t=t+h. Will be used in the next iteration 
    end

         if (safe_flag_traj==0)
            h2a = plot (X(1, :), X(2, :), 'r', 'LineWidth',1);
            h2a.Color(4)=0.3;
        else
            h2a = plot (X(1, :), X(2, :), 'b', 'LineWidth',1);
            h2a.Color(4)=0.3;
         end
end

fail_prob = fail_cnt/traj_num

figname = ['eta=',num2str(eta),'_lambda =',num2str(lambda), '.jpg'];
% figname = ['eta=',num2str(eta),'_no_theta.fig'];
saveas(gcf,figname)