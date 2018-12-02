function [ J_opt, u_opt_ind ] = LinearProgramming( P, G )
%LINEARPROGRAMMING Value iteration
%   Solve a stochastic shortest path problem by Linear Programming.
%
%   [J_opt, u_opt_ind] = LinearProgramming(P, G) computes the optimal cost
%   and the optimal control input for each state of the state space.
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
u_opt_ind = zeros(K,1);


%re - arrange matrix G and P
G_vec = [];
P_red = [];
I = [];

for L = 1:length(G(1,:))
    G_vec = [G_vec; G(:,L)];
    P_red = [P_red; P(:,:,L)];
    I = [I; eye(K)];
end

%avoid obtaining NaN values in the linprog solver
inf_index = find(isinf(G_vec));
G_vec(inf_index) = 10^5;


% disp(size(I))
% disp(size(P_red))
% disp(size(G_vec))

A = I - P_red;
f = -1 * ones(1, K);
J_opt = linprog(f, A, G_vec);


for k = 1:K
    P_k = squeeze(squeeze(P(k, :, :)));
    [val, u_opt_ind(k)] = min(G(k, :) + J_opt' * P_k );
end


end

