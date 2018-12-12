function G = ComputeStageCosts( stateSpace, controlSpace, map, gate, mansion, cameras )
%COMPUTESTAGECOSTS Compute stage costs.
% G_mine = ComputeStageCosts( stateSpace, controlSpace, map, gate, mansion, cameras );
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
        G(k,5) = 1;
    end
end

DispVec = [G stateSpace];%debug

%map of states with 0 if not on the line or column of camera or P_busted
%if on same line
BustedMap = zeros(K,1);
for k = 1:length(BustedMap)
    
    %check if stateSpace is a pond
    [a, b] = find(map<0);
    ISPOND = ismember(stateSpace(k, :), [a b], 'rows');
    
    cam_index = 0; %index for the following array
    Prob_array_cam = []; %array of cameras that see the phtographer
    
    for c = 1:size(cameras(:,1))
        quality = cameras(c,3);
        
        %check if point is on the same column as camera
        if stateSpace(k,1) == cameras(c,1)
            
            %distance camera - paparazzi
            distance = sqrt((stateSpace(k,1) - cameras(c,1))^2 + (stateSpace(k,2) - cameras(c,2))^2);
            
            %check for obstacles in each row (m) along the column bw the
            %point and camera
            OBST = false;
            for m = (min(cameras(c,2),stateSpace(k,2))+1):(max(cameras(c,2),stateSpace(k,2))-1)
                %if obstacle in bw, set quality to 0
                if any( map(m, stateSpace(k,1)) > 0 )
                    OBST = true;
                    break
                end
            end
            
            if OBST == false
                cam_index = cam_index + 1;
                Prob_array_cam(cam_index) = quality / distance; %still need to check if pond and multiple
            end
            
        end
        
        
        %check if point is on the same line as camera
        if  stateSpace(k,2) == cameras(c,2)
            
            %distance camera - paparazzi
            distance = sqrt((stateSpace(k,1) - cameras(c,1))^2 + (stateSpace(k,2) - cameras(c,2))^2);
            
            
            OBST = false;
            for m = (min(cameras(c,1),stateSpace(k,1))+1):(max(cameras(c,1),stateSpace(k,1))-1)
                
                if any( map(stateSpace(k,2),m) > 0 )
                    %no need to check other rows if an obstacle is found,
                    %you can break (exit) the loop
                    OBST = true;
                    break
                end
                
            end
            
            if OBST == false
                cam_index = cam_index + 1;
                Prob_array_cam(cam_index) = quality / distance; % still need to check if pond and multiple
            end
            
        end
        
    end %end of cameras loop
    
    if Prob_array_cam %if there is at least a camera
        camNumber = length(Prob_array_cam);
        
        if camNumber > 4
            print('Too many cameras ahaha') %something is wrong
        end
        
        %if P is not full, set remaining elements to 0
        % i. e. P = [0.4 0.8 0 0]
        for p = 1:4
            if camNumber < p
                Prob_array_cam(p) = 0;
            end
        end
        
        P_final = Prob_array_cam(1)...
            + (1 - Prob_array_cam(1)) * Prob_array_cam(2)...
            + (1 - Prob_array_cam(1)) * (1 - Prob_array_cam(2)) * Prob_array_cam(3) ...
            + (1 - Prob_array_cam(1)) * (1 - Prob_array_cam(2)) * (1 - Prob_array_cam(3)) * Prob_array_cam(4);
        
        %if the paparazzi is on the pond, prob of getting caught increases
        if ISPOND
            P_final = P_final...
                + (1 - P_final) * P_final...
                + (1 - P_final) * (1 - P_final) * P_final ...
                + (1 - P_final) * (1 - P_final) * (1 - P_final) * P_final;
        end
        
        BustedMap(k) = P_final;
    end
    
    if BustedMap(k) > 1
        disp('Busted map (k) > 1, something is wrong');
    end
end


% Result = [BustedMap stateSpace] %uncomment to check the above function
BustedCost = 7;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Picture of Paparazzi
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
PictureOfCelebCost = 1; % Assuming to be the termination cost
% PictureMap = zeros(K,1);
%Fill Cost Matrix G
iter = 0;
for k = 1:K %iter over each point of state space
    iter = iter +1;
    
    for p = 1:L %iter over each command (matrix layer)
        
        if p == 1 %north
            if G(k,p) ~= Inf %if not inf, i.e. if accessible
                
                %find index of stateSpace of the cell where I am going to
                %move
                ind = find(stateSpace(:,2) == stateSpace(k,2)+1 & stateSpace(:,1) == stateSpace(k,1));
                
                G(k,p) = G(k,p)*(1-BustedMap(ind)) ...
                    + BustedCost* BustedMap(ind);
            end
        end
        
        if p == 2 %west
            if G(k,p) ~= Inf
                ind = find(stateSpace(:,1) == stateSpace(k,1)-1 & stateSpace(:,2) == stateSpace(k,2));
                G(k,p) = G(k,p)*(1-BustedMap(ind)) ...
                    + BustedCost * BustedMap(ind);
            end
        end
        
        if p == 3 %south
            if G(k,p) ~= Inf
                ind = find(stateSpace(:,2) == stateSpace(k,2)-1 & stateSpace(:,1) == stateSpace(k,1));
                G(k,p) = G(k,p)*(1 - BustedMap(ind)) ...
                    + BustedCost * BustedMap(ind);
            end
        end

        if p == 4 %east
            if G(k,p) ~= Inf
                ind = find(stateSpace(:,1) == stateSpace(k,1)+1 & stateSpace(:,2) == stateSpace(k,2));
                G(k,p) = G(k,p)*(1- BustedMap(ind)) ...
                    + BustedCost * BustedMap(ind);
            end
        end
        
        if p == L
            if G(k,p) ~= Inf
                G(k,p) = PictureMap(k) * PictureOfCelebCost ... %take pic celebrity, this line = 0
                    + (1 - PictureMap(k)) * BustedMap(k) * BustedCost ... %fail to take pic and get busted
                    + (1 - PictureMap(k)) * (1 - BustedMap(k)) * G(k,p); %fail to take pic and not get busted

            end
        end
    end
end


DispVec = [G(:,5), stateSpace(:, :)]; %for debugging


end
