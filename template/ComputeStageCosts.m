function G = ComputeStageCosts( stateSpace, controlSpace, map, gate, mansion, cameras )
%COMPUTESTAGECOSTS Compute stage costs.
% 	Compute the stage costs for all states in the state space for all
%   control inputs.
%
%   G = ComputeStageCosts(stateSpace, controlSpace, map, gate, mansion,
%   cameras) computes the stage costs for all states in the state space for
%   all control inputs.
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
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the expected stage cost if we are in state i and
%           apply control input l.

% put your code here
K = length(stateSpace);
L = length(controlSpace);
G = inf(K, L);

%Extend map with walls
extendMap = ones(size(map)+2);
for i = 1:size(map,1)
    for j = 1:size(map,2)
        value = map(i,j);
        extendMap(i+1,j+1) = value;        
    end
end

%Fil BaselineG
for k = 1:K
    %find position on extendedMap
    PosX = stateSpace(k, 1) + 1;
    PosY = stateSpace(k, 2) + 1;
    
    %north
    if extendMap(PosY + 1, PosX) == 0
        G(k,1) = 1;
    elseif extendMap(PosY + 1, PosX) < 0
        G(k,1) = 4;
    end
        
    %south
    if extendMap(PosY - 1 , PosX) == 0
        G(k,3) = 1;
    elseif extendMap(PosY - 1, PosX) < 0
        G(k,3) = 4;
    end
    
    %west
    if extendMap(PosY, PosX - 1) == 0
        G(k,2) = 1;
    elseif extendMap(PosY, PosX - 1) < 0
        G(k,2) = 4;
    end
    
    %east
    if extendMap(PosY, PosX + 1) == 0
        G(k,4) = 1;
    elseif extendMap(PosY, PosX + 1) < 0
        G(k,4) = 4;
    end
    
    %photo
    if extendMap(PosY, PosX) == 0
        G(k,5) = 1;
    elseif extendMap(PosY, PosX) < 0
        G(k,5) = 4;
    end
end


%States with disturbances in cost due to external camera Busted
BustedProbabilityMap = BustedMapCreator(stateSpace, map, cameras);
BustedCost = 6;

%Picture of Paparazzi
PictureOfCelebrityCost = 0; % Assuming to be the termination cost
PictureProbabiltyMap = PictureMapCreator(stateSpace, map, mansion);

%Fill G
for k = 1:K
    for l = 1:(L-1) 
        if G(k,1) == Inf
            continue
        end
        G(k,l) = G(k,l)*(1-BustedProbabilityMap(k)) ...
                + BustedCost*BustedProbabilityMap(k);
    end
    for l = L
        if G(k,1) == Inf
            continue
        end
        G(k,l) = ( G(k,l)*(1-PictureProbabiltyMap(k)) + PictureOfCelebrityCost*PictureProbabiltyMap(k) )*(1-BustedProbabilityMap(k)) ...
                + BustedCost*BustedProbabilityMap(k);
    end
end







end
