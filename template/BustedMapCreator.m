function BustedMap = BustedMapCreator(stateSpace, map, cameras)

K = length(stateSpace);

BustedMap = zeros(K,1);
for k = 1:K
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
                      
            %distance camera - paparazzi
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


end