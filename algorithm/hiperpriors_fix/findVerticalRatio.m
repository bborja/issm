function [razmSky, razmLand, razmSea] = findVerticalRatio(hParamK, hParamN, sizeY, sizeX, epsilon, tip)
    %Sky : Land : Sea
    %[0, sky] = sky, [sky, land] = land & [land, 1] = sea
    
    if(tip == 1)
        pC = hParamK*(sizeX/2) + hParamN; %center of horizontal line
        razmSky = (pC / 2) / sizeY; %center od sky je na polovico zgoraj od horizonta
        razmLand = (pC + ((sizeY - pC)/2)) / sizeY; %center od morja je na polovici pod horizontom
        razmSea = 0;
    elseif(tip == 2)
        [razmSky, razmLand, razmSea] = findVerticalRatio2(hParamK, hParamN, sizeY, sizeX, epsilon);
    else
        error('napacen tip razmerja priorju');
    end

end

function [razmSky, razmLand, razmSea] = findVerticalRatio2(hParamK, hParamN, sizeY, sizeX, epsilon)
	pL = hParamN;
	pR = hParamK*sizeX + hParamN;
	
	if(pL < pR)
		minHL = pL;
		maxHL = pR;
	else
		minHL = pR;
		maxHL = pL;
	end
	
	razmSky = (minHL / sizeY) - epsilon;
	razmLand = (minHL / sizeY) + epsilon;
	razmSea = (maxHL / sizeY);
	
end