% =========================================================================
% Simulation settings
% =========================================================================
% Related Journal Reference: 
% [1] Q.-V. Pham, "UAV Communicationsfor Sustainable Federated Learning", 
% 	  IEEE Trasactions on Vehicular Technology, 2021. 
%     DOI: 10.1109/TVT.2021.3065084
%
% [2] Q.-V. Pham, "UAV-based Wireless Powered Communication for 
%     Energy-Efficient Federated Learning", prepared for GLOBECOM-2021
%
% [3] Q.-V. Pham, "Energy-Efficient Federated Learning over UAV-based
%     Wireless Powered Communication", IEEE TVT, 2021 (under review).
%
% COPYRIGHT NOTICE:
% All rights belong to Quoc-Viet Pham (email: vietpq90@gmail.com).
% This simulation code can be freely modified and distributed with the 
% original copyright notice. 
% Using this code with your own risk.
%
% Author: QUOC-VIET PHAM
% E-Mail: vietpq90@gmail.com / vietpq@pusan.ac.kr
% Created: 2020 Nov 11
% Current: 2021 Sept 29
% =========================================================================
function sim_para = paras_sim_lin
    % default parameters
    sim_para.K = 20;
    % sim_para.K = 28;
    % sim_para.K = 30; 
    % sim_para.K = 35;
    % sim_para.K = 40; 
    % sim_para.K = 50; 
    % sim_para.K = 60; 
    sim_para.alpha = 2.01;
    % sim_para.alpha = 2.1;
    % sim_para.C = 8; sim_para.H = 2;
    % sim_para.C = 25; sim_para.H = 5;
    sim_para.C = 20; sim_para.H = 5;
    % sim_para.B = 1e6;
    % sim_para.B = 2e6;
    % sim_para.B = 5e6;
    sim_para.B = 10e6;
    % sim_para.B = 40e6;
    % sim_para.B = 80e6;
    sim_para.n0 = db2lin(-174 - 30);
    % sim_para.n0 = db2lin(-169 - 30);
    % sim_para.n0 = db2lin(-155 - 30);
    % sim_para.n0 = db2lin(-150 - 30);
    % sim_para.n0 = db2lin(-120 - 30);
    % sim_para.n0 = db2lin(-90 - 30);
    % sim_para.n0 = db2lin(-80 - 30);    % TVT Correspondence
    % sim_para.n0 = db2lin(-100 - 30);
    % sim_para.beta0_h = 1.42e-4*2.2846;
    % sim_para.beta0_h = 1.42e-3*2.2846;
    % sim_para.beta0_h = 1e-3;
    % sim_para.beta0_h = 1e-2;    
    % sim_para.beta0_h = 1.42e-2*2.2846;
    % sim_para.beta0_h = db2lin(-5.0);
    sim_para.beta0_h = db2lin(-3.0);
    % sim_para.beta0_h = 1.42e-4;
    
    %%% power allocation
    % sim_para.P_UAV = db2lin(38-30);
    % sim_para.Pmax_UAV = db2lin(38-30);
    sim_para.Pmax_UAV = db2lin(30-30);
    sim_para.Pmin_UAV = 0;
    % sim_para.Pmax_User = db2lin(5-30);
    sim_para.Pmax_User = db2lin(10-30);
    sim_para.Pmin_User = 0;
    
    % Tra sis TWC paper 2021
    % sim_para.Pmax_User = db2lin(5-30);
    % sim_para.Pmin_User = 0;
    % sim_para.Pmax_User = (6 + rand*(10-6))*1e-3;
    % sim_para.Pmin_User = (0 + rand*(2-0))*1e-3;
    
    
    sim_para.eta0 = 0.9;
    % sim_para.eta0 = 0.5;
    sim_para.zeta = 1e-28;
    % sim_para.fmax = 1.0e9;
    % sim_para.fmax = 2.0e9;
    sim_para.fmax = 2.0;
    % sim_para.fmin = 0.50e9;
    % sim_para.fmin = 0.25e9;
    % sim_para.fmin = 0.10e9;
    sim_para.fmin = 0;
    % sim_para.s = 100e3;
    % sim_para.s = 90e3;
    % sim_para.s = 80e3;
    % sim_para.s = 70e3;
    % sim_para.s = 60e3;
    sim_para.s = 50e3;     % transmit data size
    % load rands.mat;
    % sim_para.s = 80e3 + (120 - 80)*1e3*rands(1,1:sim_para.K);
    % sim_para.s = 28.1e3;    % [Zhaohui Yang TWC paper]
    % sim_para.s = 1e3;
    % sim_para.s = 1e3;
    % load('CDk.mat');
    load('CkDk.mat');
    sim_para.Dk_all = Dk;
    sim_para.Ck_all = Ck;
    sim_para.Dk = Dk(1,1:sim_para.K);
    sim_para.Ck = Ck(1,1:sim_para.K);
    % Dk = (5 + 5*rand(1,sim_para.K))*1e6;
    % Ck = 10 + 10*rand(1,sim_para.K);
    % save('CDk.mat','Dk','Ck');
    % Dk = 500*ones(1,sim_para.K);
    % Ck = (1 + 2*rand(1,sim_para.K))*1e4;
    % save('CkDk.mat','Dk','Ck');
    % sim_para.Nk = 4;
    sim_para.consecutive = 2;
    sim_para.T = 50;
    % sim_para.T = 100;
    % sim_para.T = 150;
    % sim_para.T = 300;
    % sim_para.T = 1000;
    sim_para.Tol = 1e-3;
    sim_para.Tol1 = 1e-4;
    sim_para.Tol2 = 1e-5;
    % sim_para.rho = 0.5;
    % sim_para.rho = 5e-3;
    % sim_para.rho = 1e-2;
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
    % sim_para.M = 24e-3; sim_para.sigma = 6400; sim_para.iota = 0.003;
    sim_para.M = 24e-3; sim_para.sigma = 1500; sim_para.iota = 0.0014;
    
    % Federated learning model
    sim_para.gamma = 2; sim_para.L = 4;
    sim_para.xi = 1/3; sim_para.delta = 1/4;
    % sim_para.xi = 1/10; sim_para.delta = 1/10;
    % sim_para.eta = 0.5;   % it is an optimization variable!
    % sim_para.epsilon0 = 1e-3;
    sim_para.epsilon0 = 1e-2;
    sim_para.varepsilon = 1e-3;
    
    %%% FL model
    sim_para.a = 2*sim_para.L^2 * log(1/sim_para.epsilon0) ...
        / (sim_para.gamma^2 * sim_para.xi);
    sim_para.nu = 2 / ((2 - sim_para.L * sim_para.delta) ...
        * sim_para.delta * sim_para.gamma);
end