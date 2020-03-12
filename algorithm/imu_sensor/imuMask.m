function [maskSeaL, maskSeaR, maskSkyL, maskSkyR] = imuMask(hParam, sims, sim, masks_to_generate)
    sigma_value = [1, 1]; %sigma_x = 4, sigma_y = 1
    maskSeaL = zeros(sims(1), sims(2));
    maskSeaR = zeros(sims(1), sims(2));
    maskSkyL = zeros(sims(1), sims(2));
    maskSkyR = zeros(sims(1), sims(2));
	
	factor_height = sims(1) / sim(1);
    
    x = [0, 0, sims(2), sims(2)];
    
    y1_Sea = [sims(1), hParam(2) * factor_height, (hParam(1) * sim(2) + hParam(2)) * factor_height, sims(1)];
    y1_Sky = [0, hParam(2) * factor_height, (hParam(1) * sim(2) + hParam(2)) * factor_height, 0];
    y2_Sea = [sims(1), hParam(4) * factor_height, (hParam(3) * sim(2) + hParam(4)) * factor_height, sims(1)];
    y2_Sky = [0, hParam(4) * factor_height, (hParam(3) * sim(2) + hParam(4)) * factor_height, 0];
    
    %build masks and smoothen them with gaussian filter
    if(masks_to_generate == 1 || masks_to_generate == 3)
        maskSeaL = (double(poly2mask(x, y1_Sea, sims(1), sims(2))));
        maskSeaR = (double(poly2mask(x, y2_Sea, sims(1), sims(2))));
        mask_tmp_l = imgaussfilt(maskSeaL, sigma_value);
        mask_tmp_r = imgaussfilt(maskSeaR, sigma_value);
        
        maskSeaL = maskSeaL .* mask_tmp_l;
        maskSeaR = maskSeaR .* mask_tmp_r;

    end
    if(masks_to_generate == 2 || masks_to_generate == 3)
        maskSkyL = double(poly2mask(x, y1_Sky, sims(1), sims(2)));
        maskSkyR = double(poly2mask(x, y2_Sky, sims(1), sims(2)));
        mask_tmp_l = imgaussfilt(maskSkyL, sigma_value);
        mask_tmp_r = imgaussfilt(maskSkyR, sigma_value);
        
        maskSkyL = maskSkyL .* mask_tmp_l;
        maskSkyR = maskSkyR .* mask_tmp_r;
    end

end