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