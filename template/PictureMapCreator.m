function PictureMap = PictureMapCreator(stateSpace, map, mansion)

K = length(stateSpace);

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
end