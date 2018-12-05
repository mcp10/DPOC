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
P = zeros(K, K, length(controlSpace));
%get position of gate in stateSpace vector
[ret gatePos] = ismember(gate, stateSpace, 'rows');

%map of states with 0 if not on the line or column of camera or P_busted
%if on same line
BustedMap = zeros(K,1);

for k = 1:length(BustedMap)
    for c = 1:size(cameras(:,1))
        %check if point is on the same column as camera
        if stateSpace(k,1) == cameras(c,1)
            quality = cameras(c,3);
            
            %check if stateSpace is a pond
            [a, b]=find(map<0);
            ret2 = ismember(stateSpace(k, :), [a b], 'rows');
            if ret2 %if pond, increase prob of cameras by 4
                quality = 4 * cameras(c,3);
            end
            
            %distance camera - man
            distance = sqrt((stateSpace(k,1) - cameras(c,1))^2 + (stateSpace(k,2) - cameras(c,2))^2);
            %check for obstacles in each row (m) along the column bw the
            %point and camera
            for m = (min(cameras(c,2),stateSpace(k,2))+1):(max(cameras(c,2),stateSpace(k,2))-1)
                %if obstacle in bw, set quality to 0
                if map(m, stateSpace(k,1)) > 0
                    quality = 0;
                    break
                end
            end
            
            % apply probability
            BustedMap(k) = BustedMap(k) + quality / distance;
        end
        
        % same comments as above for this for loop
        if  stateSpace(k,2) == cameras(c,2)
            quality = cameras(c,3);
            distance = sqrt((stateSpace(k,1) - cameras(c,1))^2 + (stateSpace(k,2) - cameras(c,2))^2);
            for m = (min(cameras(c,1),stateSpace(k,1))+1):(max(cameras(c,1),stateSpace(k,1))-1)
                if map(stateSpace(k,2),m)>0
                    quality = 0;
                    %no need to check other rows if an obstacle is found,
                    %you can just break (exit) the loop
                    break
                end
            end
            % apply probability
            BustedMap(k) = BustedMap(k) + quality / distance;
        end
    end
end
% Result = [BustedMap stateSpace] %uncomment to check the above function

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

PictureMap = 0.001*ones(K,1);

for k = 1:K
    for m = 1:size(mansion(:,1))
        %check if point is on the same column as mansion
        if stateSpace(k,1) == mansion(m,1)
            quality = 0.5;
            
            %distance paparazzi - mansion
            distance = sqrt((stateSpace(k,1) - mansion(m,1))^2 + (stateSpace(k,2) - mansion(m,2))^2);
            %check for obstacles in each row (m) along the column bw the
            %point and camera
            for n = (min(mansion(m,2),stateSpace(k,2))+1):(max(mansion(m,2),stateSpace(k,2))-1)
                %if obstacle in bw, set quality to 0
                if map(n, stateSpace(k,1)) > 0
                    quality = 0;
                    break
                end
            end
            
            % apply probability
            PictureMap(k) = max(PictureMap(k), quality / distance);
        end
        
        % same comments as above for this for loop
        if  stateSpace(k,2) == mansion(m,2)
            quality = 0.5;
            distance = sqrt((stateSpace(k,1) - mansion(m,1))^2 + (stateSpace(k,2) - mansion(m,2))^2);
            for n = (min(mansion(m,1),stateSpace(k,1))+1):(max(mansion(m,1),stateSpace(k,1))-1)
                if map(stateSpace(k,2),n)>0
                    quality = 0;
                    %no need to check other rows if an obstacle is found,
                    %you can just break (exit) the loop
                    break
                end
            end
            % apply probability
            PictureMap(k) = max(PictureMap(k), quality / distance);
        end
    end
end

DispVec = [PictureMap, stateSpace(:, :)]; %debug
disp(PictureMap(64))

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%double for loop, analysing all transition probabilities from k1 to k2
for k1 = 1:K
    for k2 = 1:K
        %if on same column and if 1 step above (layer n)
        if stateSpace(k1, 1) == stateSpace(k2, 1) && stateSpace(k2,2) == stateSpace(k1, 2) + 1
            P(k1,k2,1) = 1 - BustedMap(k2);
            P(k1,gatePos,1) = BustedMap(k2);  %prob of being sent to the gate
        end
        
        %if on same row and 1 step before (layer w)
        if stateSpace(k1, 2) == stateSpace(k2, 2) && stateSpace(k2,1) == stateSpace(k1, 1) - 1
            P(k1,k2,2) = 1 - BustedMap(k2);
            P(k1,gatePos,2) = BustedMap(k2);  %prob of being sent to the gate
        end
        
        %if on same column and if 1 step below (layer s)
        if stateSpace(k1, 1) == stateSpace(k2, 1) && stateSpace(k2,2) == stateSpace(k1, 2) - 1
            P(k1,k2,3) = 1 - BustedMap(k2);
            P(k1,gatePos,3) = BustedMap(k2);  %prob of being sent to the gate
        end
        
        %if on same row and 1 step after (layer e)
        if stateSpace(k1, 2) == stateSpace(k2, 2) && stateSpace(k2,1) == stateSpace(k1, 1) + 1
            P(k1,k2,4) = 1 - BustedMap(k2);
            P(k1,gatePos,4) = BustedMap(k2);  %prob of being sent to the gate
        end
        
        %taking a picture
        %if on same cell
        if k1 == k2
            P(k1,k2,5) = 1 - (BustedMap(k2) * (1 - PictureMap(k1))) - (PictureMap(k1)); %need to include (- prob of catching celebrity) here?
        end
        
    end
    P(k1,gatePos,5) = BustedMap(k1) * (1 - PictureMap(k1));
    if k1 == gatePos
        P(k1,gatePos,5) = (1 - PictureMap(k1));
    end
    
end


end