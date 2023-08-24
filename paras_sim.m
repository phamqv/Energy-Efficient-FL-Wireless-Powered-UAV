% =========================================================================
% Simulation settings
% =========================================================================
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
% =========================================================================
function sim_para = paras_sim
    % default parameters
    sim_para.K = 50; 
    sim_para.alpha = 2.01;
    sim_para.C = 10; sim_para.H = 5;
    sim_para.B = 20e6;
    sim_para.n0 = db2lin(-174 - 30);
    sim_para.beta0_h = db2lin(-10.0);
    
    % power allocation
    sim_para.Pmax_UAV = db2lin(30-30);
    sim_para.Pmin_UAV = 0;
    sim_para.Pmax_User = db2lin(10-30);
    sim_para.Pmin_User = 0; 
    
    sim_para.eta0 = 0.9;
    sim_para.zeta = 1e-28;
    sim_para.fmax = 2.0;
    sim_para.fmin = 0;
    sim_para.s = 50e3;     % transmit data size
    load('CkDk.mat');
    sim_para.Dk_all = Dk;
    sim_para.Ck_all = Ck;
    sim_para.Dk = Dk(1,1:sim_para.K);
    sim_para.Ck = Ck(1,1:sim_para.K);
    sim_para.consecutive = 2;
    sim_para.T = 50;
    % sim_para.T = 100;
    sim_para.Tol = 1e-3;
    sim_para.Tol1 = 1e-4;
    sim_para.Tol2 = 1e-5;
    sim_para.rho = 1e-1;
    sim_para.div = 20;
    
    % BSUM
    sim_para.Tscale_1 = 1;
    sim_para.Tscale_11 = 5;
    sim_para.Tscale_12 = 1;
    sim_para.Tscale_2 = 2;
    sim_para.Tscale_5 = 5;
    sim_para.Tscale_10 = 10;
    
    % Energy harvesting model
    sim_para.M = 24e-3; sim_para.sigma = 1500; sim_para.iota = 0.0014;
    
    % Federated learning model
    sim_para.gamma = 2;
    sim_para.L = 4;
    sim_para.xi = 1/3; sim_para.delta = 1/4;
    sim_para.epsilon0 = 1e-3;
    % sim_para.epsilon0 = 1e-2;
    sim_para.varepsilon = 1e-3;
    
    %%% FL model
    sim_para.a = 2*sim_para.L^2 * log(1/sim_para.epsilon0) ...
        / (sim_para.gamma^2 * sim_para.xi);
    sim_para.nu = 2 / ((2 - sim_para.L * sim_para.delta) ...
        * sim_para.delta * sim_para.gamma);
end