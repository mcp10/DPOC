%check differences in the results obtained with the three methods

[ J_opt_VI, u_opt_ind_VI ] = ValueIteration( P, G );

% [ J_opt_PI, u_opt_ind_PI ] = PolicyIteration( P, G );

[ J_opt_LP, u_opt_ind_LP ] = LinearProgramming( P, G );

% J_opt_VI_PI = norm(J_opt_VI - J_opt_PI)
% u_opt_difference_VI_PI = norm(u_opt_ind_VI - u_opt_ind_PI)

J_opt_VI_LP = norm(J_opt_VI - J_opt_LP)
u_opt_difference_VI_LP = norm(u_opt_ind_VI - u_opt_ind_LP)

K = length(P(:, 1, 1));

u_opt_ind_LP = [];
for k = 1:K
    P_k = squeeze(squeeze(P(k, :, :)));
    [val, u_opt_ind_LP(k)] = min(G(k, :) + J_opt_LP' * P_k );
end

u_opt_ind_VI = [];
for k = 1:K
    %transition prob matrix (K x L): P_k(j,l)is the trans prob
    %from state k to state j, if input l is applied
    P_k = squeeze(squeeze(P(k, :, :)));
    %the matrix product below represents the SUM operator of the VI
    %formula
    [val, u_opt_ind_VI(k)] = min(G(k, :) + J_opt_VI' * P_k );
end

J_opt_VI_LP = norm(J_opt_VI - J_opt_LP)
u_opt_difference_VI_LP = norm(u_opt_ind_VI - u_opt_ind_LP)



% J_opt_LP_PI = norm(J_opt_LP - J_opt_PI)
% u_opt_difference_LP_PI = norm(u_opt_ind_LP - u_opt_ind_PI)
