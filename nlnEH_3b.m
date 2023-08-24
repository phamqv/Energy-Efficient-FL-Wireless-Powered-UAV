%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% the proposed PPTABC for the nonlinear EH model
% 
% Related Journal Reference: 
% [1] Q.-V. Pham, "UAV Communicationsfor Sustainable Federated Learning", 
% 	  IEEE Trasactions on Vehicular Technology, vol. 70, no. 4, pp. 3944-3948, Apr. 2021.
%     DOI: 10.1109/TVT.2021.3065084
%
% [2] Q.-V. Pham, "UAV-based Wireless Powered Communication for Energy-Efficient Federated Learning", 
%     in IEEE International Conference on Communications (ICC), Seoul, Korea, May 2022.
% 	  10.1109/ICC45855.2022.9838414
%
% [3] Q.-V. Pham, "Energy-Efficient Federated Learning over UAV-based
%     Wireless Powered Communication", IEEE Transactions on Vehicular Technology, 
%	  vol. 71, no. 5, pp. 4977-4990, May 2022.
% 	  DOI: 10.1109/TVT.2022.3150004
%
% Name:  Quoc-Viet Pham
% email: vietpq90@gmail.com
% Created:  2021 / 03 / 18
% Current:  2023 / 08 / 24
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [t,f,P,p,b,q,eta,obj_cur,conv_cur,cv_curve] = nlnEH_3b(sim_para,post_UE)
    % parameters
    rho = sim_para.rho;
    T = sim_para.T;
    sigma = sim_para.sigma;
    iota = sim_para.iota;
    nuk = 1./(1 + exp(sigma .* iota));
    psi_0 = sim_para.T * sim_para.M ./ (1 - nuk);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % randomly initialize feasible postions
    b = 1.0 * ones(1,sim_para.K) * sim_para.B / sim_para.K;    
    p = 0.2 * ones(1,sim_para.K) * sim_para.Pmax_User;
    q = sim_para.C*[-0.3443, 0.3569];
    u = zeros(1,sim_para.K);
    for k = 1:sim_para.K
        u(k) = (sim_para.H^2 ...
            + norm(q - post_UE(k,:),2)^2)^(sim_para.alpha/2);
    end
    eta = 0.80;
    CkDk = sim_para.Ck.*sim_para.Dk*1e-9;
    zetaCkDk = sim_para.zeta*sim_para.Ck.*sim_para.Dk*(1e9)^2;
    P = sim_para.Pmax_UAV;
    g0 = sim_para.beta0_h / sim_para.n0; 
    t = 1.0*(sim_para.s ./ (b.*log2(1 + p*g0./(b .* u))));
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    i = 1.0*(1 + exp(-sigma * (P*sim_para.beta0_h./u - iota)));
    w = -1.0*sigma.*(P*sim_para.beta0_h./u - iota);
    z = 1.0*(1 + exp(w));
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % f = ones(1,sim_para.K)*0.2;
    fmin = -sim_para.nu*log(eta)*CkDk./(sim_para.T*(1-eta)/sim_para.a - t);
    fmin = max(fmin, sim_para.fmin);
    fmax = psi_0.*(1./i - nuk)- t.*p;
    fmax = min(sqrt(fmax./(-sim_para.nu*log(eta)*zetaCkDk)),sim_para.fmax);    
    f = (fmin + fmax)/10;

    % declare variables and parameters
    flag = 0;
    iter = 1; 
    Obj = sum(sim_para.a/(1-eta) ...
        * (sim_para.nu*log(1/eta)*zetaCkDk.*power(f,2)+t.*p));
    obj_pre = Obj + rho*T*P;
    obj_cur = obj_pre;
    conv_cur = zeros(1,100);
    conv_cur(iter) = obj_pre;
    % newly added
    cv_curve = zeros(2,100);
    cv_curve(1,iter) = Obj;         % total energy of FL users
    cv_curve(2,iter) = rho*T*P;     % energy consumption of the FL server     
       
    while (flag < sim_para.consecutive)
        % increase the iteration index by 1
        iter = iter + 1;            
        
        %________________________ the block (eta) ________________________%
        eta_min = 1e-4;
        eta_max = 1-1e-4;      
        B_1 = sim_para.a*(sim_para.nu*log(1/eta_max)*zetaCkDk.*power(f,2)+t.*p);
        t_l = sum(B_1)/(1-eta_min);       
        B_2 = sim_para.a*(sim_para.nu*log(1/eta_min)*zetaCkDk.*power(f,2)+t.*p);
        t_u = sum(B_2)/(1-eta_max);       
        % varepsilon = 1e-2;
        eta_var = eta;
        eta_pre = eta;
        while (t_u - t_l > sim_para.varepsilon)
            % solve the feasibility problem at its midpoint
            t_mid = (t_u + t_l)/2;
            
            % solve the feasiblity problem
            cvx_begin quiet
            % cvx_begin
            variable eta(1,1)
            % minimize (0)
            subject to
                % constraint (45b)
                (-sim_para.nu*log(eta)*CkDk./f + t) ...
                    <= sim_para.T*(1-eta)/sim_para.a;
                
                % constraint (45c) ---- approximate
                Gamma_kap = 1./i;
                -sim_para.nu*log(eta)*zetaCkDk.*power(f,2) + t.*p ...
                    <= (psi_0*Gamma_kap - psi_0*nuk)*(1-eta)/sim_para.a;
                
                % additional constraints (45e)
                sum(-sim_para.nu*log(eta)*zetaCkDk.*power(f,2) + t.*p) ...
                    <= t_mid*(1-eta)/sim_para.a;
                
                % constraint (45d)
                eta >= 0;
                eta <= 1;
            cvx_end
            
            if( strcmp( cvx_status, 'Infeasible' ) == 1 )
                t_l = t_mid;
            else
                t_u = t_mid;                
                eta_var = eta;
            end
        end
        Obj = sum(sim_para.a/(1-eta_var)*(sim_para.nu ...
            * log(1/eta_var)*zetaCkDk.*power(f,2)+t.*p));
        Obj = Obj + rho*T*P;
        if Obj <= obj_cur
            eta = eta_var;
        else
            eta = eta_pre;
            Obj = obj_cur;
        end
        fprintf('Optimal value (bisection method): %f\n', Obj);
        
        %_______________________ the block (p,b,q) _______________________%
        sindex = 1;        
        while sindex <= sim_para.Tscale_1
            % increase the scale index
            sindex = sindex + 1;
        
            % feasible solution obtained from the previous update step
            u_kap = u;
            p_kap = p;
            b_kap = b;             
            z_kap = z;            
            
            % coefficients
            psi_4 = sim_para.a./(1-eta).*t;
            psi_5 = sim_para.T*sim_para.M/(1-nuk);
            psi_3 = sim_para.a/(1-eta)*sim_para.nu ...
                * log(1/eta)*zetaCkDk.*power(f,2) + psi_5.*nuk;            
            
            cvx_begin 
            % cvx_solver mosek
            variable p(1,sim_para.K)
            variable inv_p(1,sim_para.K) 
            variable b(1,sim_para.K)
            variable q(1,2)
            variable u(1,sim_para.K)
            variable u_au(1,sim_para.K)
            variable z(1,sim_para.K)
            variable w(1,sim_para.K)
            variable app_Rate(1,sim_para.K) 
            variable app_u(1,sim_para.K) 
            variable app_z(1,sim_para.K)

            Rate = cvx(zeros(1,sim_para.K));
            Gamma_kap = cvx(zeros(1,sim_para.K));
            Gamma_z_kap = cvx(zeros(1,sim_para.K));
            for k = 1:sim_para.K
                % rate approximation
                tau = b_kap(k);
                x = u_kap(k)/(p_kap(k) * g0);
                y = b_kap(k);
                lambda = 2*tau*log(1+1/(x*y)); 
                mu = tau/(1+x*y); 
                nu = tau^2*log(1+1/(x*y));

                % data rate approximation
                Lambda_kap = 0.25*(u(k)/u_kap(k) + p_kap(k)*inv_p(k)).^2;
                Rate(k) = log2(exp(1))*(lambda ...
                    + mu * (2 - Lambda_kap - b(k)*b_kap(k)^-1) ...
                    - nu * inv_pos(b(k)));
                
                % objective function approximation
                Gamma_kap(k) = 2/u_kap(k) - u(k)/(u_kap(k)^2);
                Gamma_z_kap(k) = 2/z_kap(k) - z(k)/(z_kap(k)^2);
            end

            % objective function approximation          
            Obj = sum(sim_para.a/(1-eta) ...
                * (sim_para.nu*log(1/eta)*zetaCkDk.*power(f,2)+t.*p));
            Obj = Obj + rho*T*P;
            minimize (Obj)
            subject to
                % constraint (19f)
                p >= sim_para.Pmin_User; 
                p <= sim_para.Pmax_User;

                % constraint (19g)
                sum(b) <= sim_para.B;
                b >= 0;

                % constraint (19i)
                sum(q.^2) <= sim_para.C^2;               

                % constraint (55b)
                Rate >= app_Rate;
                t .* app_Rate >= sim_para.s;
                               
                % constraint (55c)-(55e)
                z >= 1 + exp(w);                
                Gamma_kap >= app_u;
                Gamma_z_kap >= app_z;
                psi_3 + psi_4.*p <= psi_5 .* app_z;
                w + sigma.*(P*sim_para.beta0_h.*app_u - iota) >= 0;
                
                % constraint (26d)
                u >= power(u_au, sim_para.alpha/2);
                for k = 1:sim_para.K
                    u_au(k) >= sim_para.H^2 + sum((q - post_UE(k,:)).^2);
                end

                % constraints for auxiliary variables
                for k = 1:sim_para.K
                    [p(k) 1; 1 inv_p(k)] == semidefinite(2);
                end     
            cvx_end  
            % set the objective value
            % Obj_val_pre = Obj;
        end           
        
        %_______________________ the block (t,f,P) _______________________%
        sindex = 1;    
        while sindex <= sim_para.Tscale_1
            % increase the scale index
            sindex = sindex + 1;
            i_kap = i;
                        
            cvx_begin 
            % cvx_solver mosek
            variable f(1,sim_para.K) nonnegative
            variable t(1,sim_para.K) nonnegative
            variable P(1,1) 
            variable i(1,sim_para.K) nonnegative
            variable app_i(1,sim_para.K)
            
            Gamma_kap = cvx(zeros(1,sim_para.K));
            for k = 1:sim_para.K
                % function approximation
                Gamma_kap(k) = 2/i_kap(k) - i(k)/(i_kap(k)^2);
            end
            
            % objective function approximation           
            Obj1 = sum(sim_para.a/(1-eta) ...
                * (sim_para.nu*log(1/eta)*zetaCkDk.*power(f,2) + t.*p));
            Obj = Obj1 + rho*T*P;
            minimize (Obj)
            subject to
                % constraint (29b)
                Rate = b.*log2(1 + p*g0./(b .* u));
                t .* Rate >= sim_para.s;

                % constraint (29c)
                sim_para.a / (1 - eta) ...
                    * (sim_para.nu*log(1/eta)*CkDk.*inv_pos(f) + t) ...
                    <= sim_para.T;
                
                % this makes differences between nln and lin EH models
                % constraint (37b)
                Gamma_kap >= app_i;
                sim_para.a/(1-eta) ...
                    * (-sim_para.nu*log(eta)*zetaCkDk.*power(f,2) ...
                    + t.*p ) <= psi_0*app_i - psi_0*nuk; 
                
                % constraint (37c)
                i >= 1 + exp(-sigma * (P*sim_para.beta0_h./u - iota));

                % constraint (25e)
                P <= sim_para.Pmax_UAV;
                P >= 0;

                % constraint (25h)
                f >= sim_para.fmin * ones(1,sim_para.K);
                f <= sim_para.fmax * ones(1,sim_para.K);
            cvx_end 
        end               
        
        %__________________ check the stopping tolerance _________________%        
        obj_cur = Obj;
        conv_cur(iter) = obj_cur;
        cv_curve(1,iter) = Obj1;        % total energy of FL users
        cv_curve(2,iter) = rho*T*P;     % energy of the FL server
        Tol = abs(obj_cur - obj_pre) / obj_pre;            
        fprintf('Tolerance at iteration-%d is %f\n', iter, Tol);
        if (Tol <= sim_para.Tol)
        % if (Tol <= sim_para.Tol1)
            flag = flag + 1;
        else
            flag = 0;
        end            
        % update the previous values
        obj_pre = obj_cur;        
    end
    conv_cur = conv_cur(1:iter);
    cv_curve = cv_curve(:,1:iter);
end
