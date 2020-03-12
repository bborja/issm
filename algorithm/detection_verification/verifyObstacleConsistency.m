function [objsL, objsR] = verifyObstacleConsistency(imL, imR, objs_L, objs_R, FundMat)
    %% Input
    % imL = Left Image
    % imR = Right Image
    % objs_L = Objects detected in left camera
    % objs_R = Objects detected in right camera
    % fr = Frame number
    % vidSeq = Video Sequence name
    % vidSeqID = Video Sequnce ID
    % FundMat = Fundamental Matrix of Stereo System Cameras
    
    % threshold for RGB
    threshold_match = 0.95;
    threshold_match_other = threshold_match;
    
    
    % threshold when using fhog
    %threshold_match = 0.45;
    %threshold_match_other = 0.4
    
    
    %initialize
    [sizeY, sizeX, ~] = size(imL);
               
    % number of obstacles in left camera
    nL = numel(objs_L);
    % number of obstacles in right camera
    nR = numel(objs_R);
    
    % initialize arrays
    objsL = [];
    objsR = [];
       
    
    if(nL == 0 && nR == 0) % if no obstacles are detected...
        objsL = objs_L;
        objsR = objs_R;
        
    elseif(nL > 0 && nR == 0) % if there are obstacles only in left camera
        [objsL, objsR] = matchUnmatched(imL, imR, objs_L, threshold_match_other);
        
    elseif(nL == 0 && nR > 0) %if there are obstacles only in right camera
        [objsR, objsL] = matchUnmatched(imR, imL, objs_R, threshold_match_other);
        
    elseif(nL > 0 && nR > 0)
        objsCentersL = zeros(nL, 2);
        objsCentersR = zeros(nR, 2);
        
        matchedIndicesL = [];
        matchedIndicesR = [];
        
        % get centers of obstacles in left camera
        for i = 1 : nL           
            objsCentersL(i,1) = round(objs_L(i).bounding_box(1) + objs_L(i).bounding_box(3) / 2);
            objsCentersL(i,2) = round(objs_L(i).bounding_box(2) + objs_L(i).bounding_box(4) / 2);           
        end
        
        % get centers of obstacles in right camera
        for i = 1 : nR         
            objsCentersR(i,1) = round(objs_R(i).bounding_box(1) + objs_R(i).bounding_box(3) / 2);
            objsCentersR(i,2) = round(objs_R(i).bounding_box(2) + objs_R(i).bounding_box(4) / 2);
        end
        
        % get epipolar lines of obstacles
        EpiLinesLeft = getEpipolarLines(objsCentersL, FundMat, 1);
        %EpiLinesRight = getEpipolarLines(objsCentersR, FundMat, 2);
        
        % counter of confirmed matches
        st = 1;
        
        for i = 1 : nL		
                
            % expand bounding box by 20%            
            TemplateIL = get_patch(imL, [round(objs_L(i).bounding_box(1) + (objs_L(i).bounding_box(3) / 2)), ...
                round(objs_L(i).bounding_box(2) + (objs_L(i).bounding_box(4) / 2))], 1, ...
                [round(1.2 * objs_L(i).bounding_box(3)), round(1.2 * objs_L(i).bounding_box(4))]);

            [tempHeight_objL, tempWidth_objL, ~] = size(TemplateIL);

            % get threshold for epiline based on bounding box diagonal of detected object in left camera
            TemplateDiagLeft = sqrt( tempHeight_objL^2 + tempWidth_objL^2 );
            EpiLineThreshold = 2 * TemplateDiagLeft;

            % get all objects that are inside the threshold area of
            % epiline
            nearestPoints = findNearest(EpiLinesLeft(i,:), objsCentersR, EpiLineThreshold, sizeX);
            lenNrP = numel(nearestPoints);


            % initialize for matching
            maxMatch = 0;
            maxMatchIndex = 0;


            for j = 1 : lenNrP

				% Get image for searching
				% The size is chosen as the size of template expanded
				% aditionally expanded by 15%
				TemplateIR = get_patch(imR, [round(objs_R(j).bounding_box(1) + (objs_R(j).bounding_box(3) / 2)), ...
					round(objs_R(j).bounding_box(2) + (objs_R(j).bounding_box(4) / 2))], 1, ...
					[round(3 * objs_L(i).bounding_box(3)), round(3 * objs_L(i).bounding_box(4))]);
					%[round(tempWidth_objL*1.15), round(1.15*tempHeight_objL)]);

				% get maximum match between templateIL and templateIR
				%[tempMax, ~, ~] = nccHogV2(TemplateIL, TemplateIR, figName, i, j); %matching template with fhog
				try
					[tempMax, ~, ~] = nccRGB(TemplateIL, TemplateIR); %rgb template matching
				catch err
					tempMax = 0;
				end

				% For debuging only - print maxmatch and templates
				%figure(13); clf; subplot(1,2,1), imagesc(uint8(TemplateIL)); title(sprintf('Ujemanje: %04f', tempMax)); subplot(1,2,2); imagesc(uint8(TemplateIR)); title(sprintf('Ujemanje: %04f', tempMax));
				%waitforbuttonpress;

				if(tempMax > maxMatch)
					maxMatch = tempMax;
					maxMatchIndex = j;
				end
            end


            if(maxMatch > threshold_match)
                objsL(st).bounding_box = objs_L(i).bounding_box;
                objsR(st).bounding_box = objs_R(maxMatchIndex).bounding_box;
                st = st + 1;

                matchedIndicesL = [matchedIndicesL, i];
                matchedIndicesR = [matchedIndicesR, maxMatchIndex];

            end
        end
        objsL_1 = objs_L(setdiff(1:end, matchedIndicesL));
        objsR_1 = objs_R(setdiff(1:end, matchedIndicesR));
        num_matched_L = numel(objsL);
        num_matched_R = numel(objsR);

%         [objsL_21, objsR_21] = matchUnmatched(imL, imR, objsL_1, threshold_match_other);
%         [objsR_22, objsL_22] = matchUnmatched(imR, imL, objsR_1, threshold_match_other);
        [objsL_21, objsR_21] = matchUnmatched(imL, imR, objsL_1, threshold_match_other);
        [objsR_22, objsL_22] = matchUnmatched(imR, imL, objsR_1, threshold_match_other);
        
        if(numel(objsL) > numel(objsL_21))
            objsL = removeDuplicatedDetections(objsL, objsL_21);
        else
            objsL = removeDuplicatedDetections(objsL_21, objsL);
        end
        
        if(numel(objsL) > numel(objsL_22))
            objsL = removeDuplicatedDetections(objsL, objsL_22);
        else
            objsL = removeDuplicatedDetections(objsL_22, objsL);
        end
        
        if(numel(objsR) > numel(objsR_21))
            objsR = removeDuplicatedDetections(objsR, objsR_21);
        else
            objsR = removeDuplicatedDetections(objsR_21, objsR);
        end
        
        if(numel(objsR) > numel(objsR_22))
            objsR = removeDuplicatedDetections(objsR, objsR_22);
        else
            objsR = removeDuplicatedDetections(objsR_22, objsR);
        end

%         for k = 1 : numel(objsL_21)
%             objsL(num_matched_L+k).bounding_box = objsL_21(k).bounding_box;
%         end
%         num_matched_L_2 = numel(objsL);
%         for k = 1 : numel(objsL_22)
%             objsL(num_matched_L_2+k).bounding_box = objsL_22(k).bounding_box;
%         end
%         for k = 1 : numel(objsR_22)
%             objsR(num_matched_R+k).bounding_box = objsR_22(k).bounding_box;
%         end
%         num_matched_R_2 = numel(objsR);
%         for k = 1 : numel(objsR_21)
%             objsR(num_matched_R_2+k).bounding_box = objsR_21(k).bounding_box;
%         end
    end
end


%match unmatched detected obstacles
%expand search region and search for matches
%if match is found and is close enough to the horizon than accept it
%otherwise reject the detection....
function [objs1, objs2] = matchUnmatched(im1, im2, objsU, threshold_match_other, varargin)    
    n = numel(objsU);
    objsCenters = zeros(n, 2);
    
    objs1 = [];
    objs2 = [];
    st = 1;
    
    for i = 1 : n
        
        objsCenters(i, 1) = round(objsU(i).bounding_box(1) + objsU(i).bounding_box(3) / 2);
        objsCenters(i, 2) = round(objsU(i).bounding_box(2) + objsU(i).bounding_box(4) / 2);
        
        % Get patch of detected obstacle
        Template1 = get_patch(im1, objsCenters(i,:), 1, ...
                        [round(1.5 * objsU(i).bounding_box(3)), round(1.5 * objsU(i).bounding_box(4))]);
                    
        [sTy, sTx, ~] = size(Template1);
                          
        % Image (search region) in other camera...
        % (3 times the size of template)
        Template2 = get_patch(im2, objsCenters(i,:), 1, ...
                                [round(4 * sTx), round(4 * sTy)]);
                            
        [sT2y, sT2x, ~] = size(Template2);
        
        %[maxMatch, max_y, max_x] = nccHogV2(Template1, Template2, 'placeholder', 1, 1);
		try
			[maxMatch, max_y, max_x] = nccRGB(Template1, Template2);
		catch err
			maxMatch = 0;
			max_y = 1;
			max_x = 1;
		end
        
        % Get top-left coordinate of found match in im2
        % Get x-coordinate
        temp_x = objsCenters(i,1) - sT2x / 2 + max_x(1);
        % Get y-coordinate
        temp_y = objsCenters(i,2) - sT2y / 2 + max_y(1);


        % Check if there is a match...
        if(maxMatch > threshold_match_other)
            % Add detected template...
            objs1(st).bounding_box = objsU(i).bounding_box;

            % Add found matched obstacle from other camera...
            % format of bounding box:
            %       left-top x,y and width, height
            % Width and height equal to those of resized detected template
            objs2(st).bounding_box = [round(temp_x), round(temp_y), sTx, sTy];
            
            st = st+1;
        end
    end
end

function new_objs = removeDuplicatedDetections(objs_1, objs_2)
    new_objs = [];
    num_matched = 1;
    min_overlap = 0.2;

    n_objs_1 = numel(objs_1);
    n_objs_2 = numel(objs_2);
    
    for i = 1 : n_objs_1
        new_objs(num_matched).bounding_box = objs_1(i).bounding_box;
        num_matched = num_matched + 1;
        
        bb_det_frst = objs_1(i).bounding_box;
        bb_det_frst(3) = bb_det_frst(1) + bb_det_frst(3);
        bb_det_frst(4) = bb_det_frst(2) + bb_det_frst(4);
        for j = 1 : n_objs_2
            bb_det_scnd = objs_2(j).bounding_box;
            bb_det_scnd(3) = bb_det_scnd(1) + bb_det_scnd(3);
            bb_det_scnd(4) = bb_det_scnd(2) + bb_det_scnd(4);
            
            bi = [max(bb_det_frst(1), bb_det_scnd(1)); max(bb_det_frst(2), bb_det_scnd(2)); min(bb_det_frst(3), bb_det_scnd(3)); min(bb_det_frst(4), bb_det_scnd(4))];
            iw = bi(3) - bi(1) + 1;
            ih = bi(4) - bi(2) + 1;
            if(iw > 0 && ih > 0)
                ua = (bb_det_frst(3) - bb_det_frst(1) + 1) * (bb_det_frst(4) - bb_det_frst(2) + 1) + (bb_det_scnd(3) - bb_det_scnd(1)+1) * (bb_det_scnd(4) - bb_det_scnd(2) + 1) - iw * ih;
                ov = iw * ih / ua;
                
                if( ov < min_overlap)
                    %new_objs(num_matched).bounding_box = new_objs_1(i).bounding_box;
                    new_objs(num_matched).bounding_box = objs_2(j).bounding_box;
                    num_matched = num_matched + 1;
                end
            else
                new_objs(num_matched).bounding_box = objs_2(j).bounding_box;
                num_matched = num_matched + 1;
            end
        end 
    end

end

% find points that are at max distance t from epiline
function nearest = findNearest(EpiLine, obsCenters, t, w)
    [nC, ~] = size(obsCenters);
    
    a = EpiLine(1);
    b = EpiLine(2);
    c = EpiLine(3);
    
    nearest = [];
    st = 1;
    for i = 1 : nC
            vLine1 = [0, -c/b, 0];
            vLine2 = [w, -(c/b) - a*w, 0];
            
            temp1 = vLine1 - vLine2;
            temp2 = [obsCenters(i,1), obsCenters(i,2), 0] - vLine2;

            distance = norm(cross(temp1, temp2)) / norm(temp1);

            if(distance <= t)
                nearest(st) = i;
                st = st + 1;
            end
    end
end