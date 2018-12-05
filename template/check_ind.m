for k = 1:108
    ind = find(stateSpace(:,1) == stateSpace(k,1)-1 & stateSpace(:,2) == stateSpace(k,2));
    [k, ind]
end


