function R = extractTheLargestRegion (T)
    %#codegen


    % Copy input image and clear it
    R = T;
    R(:,:,:) = 0;

    % Find connected components
    CC = replacement_bwconncomp(T, 8);

    % Find the largest component
    max_idx = -1; max_val = -inf;
    for i = 1:numel(CC),
        if CC(i).area > max_val,
            max_val = CC(i).area;
            max_idx = i;
        end
    end

    if max_idx == -1,
        return;
    end

    % FIX! - extract all connected regions with larger than 0.01 * max_val
    additional_areas_idx = [max_idx];
    for i = 1:numel(CC),
        if max_idx ~= i && CC(i).area >= 0.01*max_val,
            additional_areas_idx = [additional_areas_idx, i];
        end
    end


    % "Fill" the largest components into empty output image
    for i = 1 : numel(additional_areas_idx)
        R(CC(additional_areas_idx(i)).pixel_idx) = 1;
    end
end