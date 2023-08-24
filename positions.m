function [post_UE] = positions(sim_para)
    % position of the users (UEs)
    theta_UE = rand(1, sim_para.K) * 2 * pi;
    D_UE = sim_para.C * (0.05 + 0.95*rand(1, sim_para.K));
    post_UE = zeros(sim_para.K, 2);
    post_UE(:, 1) = (D_UE .* cos(theta_UE))';
    post_UE(:, 2) = (D_UE .* sin(theta_UE))';
end

