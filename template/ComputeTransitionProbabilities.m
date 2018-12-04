function P = ComputeTransitionProbabilities( stateSpace, controlSpace, map, gate, mansion, cameras )
%COMPUTETRANSITIONPROBABILITIES Compute transition probabilities.
% 	Compute the transition probabilities between all states in the state
%   space for all control inputs.
%
%   P = ComputeTransitionProbabilities(stateSpace, controlSpace, map, gate,
%   mansion, cameras) computes the transition probabilities between all
%   states in the state space for all control inputs.
%
%   Input arguments:
%
%       stateSpace:
%           A (K x 2)-matrix, where the i-th row represents the i-th
%           element of the state space.
%
%       controlSpace:
%           A (L x 1)-matrix, where the l-th row represents the l-th
%           element of the control space.
%
%       map:
%           A (M x N)-matrix describing the terrain of the estate map.
%           Positive values indicate cells that are inaccessible (e.g.
%           trees, bushes or the mansion) and negative values indicate
%           ponds or pools.
%
%   	gate:
%          	A (1 x 2)-matrix describing the position of the gate.
%
%    	mansion:
%          	A (F x 2)-matrix indicating the position of the cells of the
%           mansion.
%
%    	cameras:
%          	A (H x 3)-matrix indicating the positions and quality of the
%           cameras.
%
%   Output arguments:
%
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability from
%           state i to state j if control input l is applied.

% put your code here
K = length(stateSpace);
L = length(controlSpace);
P = zeros(K, K, L);

%get position of gate in stateSpace vector
[ret gatePos] = ismember(gate, stateSpace, 'rows');

%map of states with 0 if not on the line or column of camera or P_busted
%if on same line
BustedMap = BustedMapCreator(stateSpace, map, cameras);

% Result = [BustedMap stateSpace] %uncomment to check the above function

%double for loop, analysing all transition probabilities from k1 to k2
for k1 = 1:K
    for k2 = 1:K
        %if on same column and if 1 step above (layer n)
        if stateSpace(k1, 1) == stateSpace(k2, 1) && stateSpace(k2,2) == stateSpace(k1, 2) + 1
            P(k1,k2,1) = 1 - BustedMap(k2);
        end
        
        %if on same row and 1 step before (layer w)
        if stateSpace(k1, 2) == stateSpace(k2, 2) && stateSpace(k2,1) == stateSpace(k1, 1) - 1
            P(k1,k2,2) = 1 - BustedMap(k2);
        end
        
        %if on same column and if 1 step below (layer s)
        if stateSpace(k1, 1) == stateSpace(k2, 1) && stateSpace(k2,2) == stateSpace(k1, 2) - 1
            P(k1,k2,3) = 1 - BustedMap(k2);
        end
        
        %if on same row and 1 step after (layer e)
        if stateSpace(k1, 2) == stateSpace(k2, 2) && stateSpace(k2,1) == stateSpace(k1, 1) + 1
            P(k1,k2,4) = 1 - BustedMap(k2);
        end
        
        %taking a picture
        %if on same cell
        if k1 == k2
            P(k1,k2,5) = 1 - BustedMap(k2);
        end
        P(k1,gatePos,:) = BustedMap(k2); %prob of being sent to the gate
    end
end


end
