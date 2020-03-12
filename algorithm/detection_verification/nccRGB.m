function [maximumMatch, maxIndY, maxIndX] = nccRGB(T, Im)
    % T - template in RGB
    % Im - image in RGB
    [sizeTy, sizeTx, ~] = size(T);
    [sizeSy, sizeSx, ~] = size(Im);
    
    %% scale template and image
    widthT = 35;
    ratio = widthT / sizeTx;
    % scale template to width 35, while keeping the ratio of the template
    T = imresize(T, [round(ratio*sizeTy), widthT]);
    
    % check variance of template to avoid an error
    T = checkTemplateVariance(T);
    
    % scale image by widthT/sizeTx ratio...
    Im = imresize(Im, [round(ratio*sizeSy), round(ratio*sizeSx)]);
    
    %% NCC matching on RGB channels....
    nccAvg = [];
    for i = 1 : 3 %across all 3 color channels...
       tempResult = normxcorr2(T(:,:,i), Im(:,:,i));
       
       if(isempty(nccAvg))
           nccAvg = tempResult * 1/3;
       else
           nccAvg = nccAvg + (tempResult * 1/3);
       end
       
    end
    
    %% Get location (x,y) position of maximum match and maximum match value
    [ypeak, xpeak] = find(nccAvg==max(nccAvg(:)));
    yoffSet = ypeak-size(T,1);
    xoffSet = xpeak-size(T,2);
    
    maximumMatch = max(nccAvg(:));
    
    
    maxIndX = round(xoffSet * 1 / ratio) + 1;
    maxIndY = round(yoffSet * 1 / ratio) + 1;
end


function template = checkTemplateVariance(tmp)
    t = double(tmp);
    tVariance = var(t(:));
    
    % is it zero? (when normxcorr2 gives us an error)
    if(tVariance == 0)
        % give random noise to template
        tmp(1,1,:) = 1;
        template = tmp;
    else
        % else we don't need to change values of template
        template = tmp;
    end

end