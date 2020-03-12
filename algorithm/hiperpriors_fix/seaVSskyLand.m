function Q = seaVSskyLand(Q_sum)
    %Generates binary mask sea VS land+sky
    %[sizeY, sizeX, ~] = size(Q_sum);
    
    %sum sky and land
    Q(:,:,1) = Q_sum(:,:,3);
    Q(:,:,2) = Q_sum(:,:,1) + Q_sum(:,:,2) + Q_sum(:,:,4);
    %no probability is smaller than 0
    Q(Q<0) = 0;
end