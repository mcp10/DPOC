function [ J_opt, u_opt_ind ] = PolicyIteration( P, G )
%POLICYITERATION Value iteration
%   Solve a stochastic shortest path problem by Policy Iteration.
%
%   [J_opt, u_opt_ind] = PolicyIteration(P, G) computes the optimal cost and
%   the optimal control input for each state of the state space.
%
%   Input arguments:
%
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%
%       J_opt:
%       	A (K x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (K x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space.

% put your code here

K = length(P(:, 1, 1));
err = 10^-5; %termination condition

%variable intializations
u_opt_old = ones(K,1) * 5; %always take a pic, this should be feasible
Prob = zeros(K, K);
u_opt_ind = zeros(K,1);
q = zeros(K,1);


ERR  = 10;
iter = 0;

while abs(ERR) > err
    iter = iter + 1;
    %%%%%%%%%%%
    %%%%%%%%%%% policy evaluation
    %%%%%%%%%%%
    
    
    % solve system of eqn J(i) = q(i) + SUM (P(i,j) * J(j))
    % J = (I-Prob)\G
    
    for k1 = 1:K
        q(k1) = G(k1, u_opt_old(k1)); %vector of costs with policy u_opt_old
        for k2 = 1:K
            %matrix used for solving the system of eqn
            Prob(k1, k2) = P(k1, k2 , u_opt_old(k1));
        end
    end
    
    %replace inf with a very high number, in case you receive errors
%     inf_index = find(isinf(q));
%     q(inf_index) = 10^9;
    
    %solve the system of eqn
    J_opt = (eye(K) - Prob)\q;
    J_opt = J_opt';
%     disp(J_opt)
    
    %%%%%%%%%%%
    %%%%%%%%%%% policy Improvement
    %%%%%%%%%%%
    
    for k1 = 1:K
        %transition prob matrix (K x L): P_k(j,l)is the trans prob
        %from state k to state j, if input l is applied
        P_k = squeeze(squeeze(P(k1, :, :)));
        %evaluate new policy solving the minimum
        [val, u_opt_ind(k1)] = min(G(k1, :) + J_opt * P_k);
    end
    
    %evaluate error
    ERR = norm(u_opt_ind - u_opt_old);
    
    %policy update for next iteration
    u_opt_old = u_opt_ind;
    
    
    %%%% debugging stuff
    
    %     disp('iter')
    %     disp(iter)
    %     disp('u_opt_ind')
    %     disp(u_opt_ind)
%     disp('Error')
%     disp(ERR)
    
end

J_opt = J_opt';

fprintf('\n')
fprintf('\n')
disp('Policy Iteration, number of iterations: ')
disp(iter)
disp('Policy Iteration, final error: ')
disp(ERR)

end

