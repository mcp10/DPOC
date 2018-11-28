function [ J_opt, u_opt_ind ] = ValueIteration( P, G )
%VALUEITERATION Value iteration
%   Solve a stochastic shortest path problem by Value Iteration.
%
%   [J_opt, u_opt_ind] = ValueIteration(P, G) computes the optimal cost and
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
J_opt_old = zeros(K,1);
J_opt = ones(K,1);
u_opt_ind = zeros(K,1);

while abs(J_opt - J_opt_old) > err
    for k = 1:K
        [J_opt(k), u_opt_ind(k)] = min(G(k, :) + J_opt_old * squeeze(squeeze(P(k, :, :)))); %errore here with squeeze
    end
    J_opt_old = J_opt;
end



end