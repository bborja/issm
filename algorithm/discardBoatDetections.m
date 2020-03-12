function [objects_L, objects_R] = discardBoatDetections(dataset_path, objs_L, objs_R, sim, videonumber)
    %% Get mask images of boat parts
    %initialize
    maskBoatL= zeros(sim(1), sim(2));
    maskBoatR = zeros(sim(1), sim(2));
    %boatMasksPath = fullfile('../../dataset/anotated-data/boatMasks');
    boatMasksPath = fullfile(dataset_path, 'annotations', 'boatMasks');
    
    %load mask if necessary
    if(videonumber >= 10 && videonumber <= 14)
        maskBoatR = rgb2gray(imread(fullfile(boatMasksPath, 'maska1R.png')));
    elseif(videonumber >= 15 && videonumber <= 18)
        maskBoatR = rgb2gray(imread(fullfile(boatMasksPath, 'maska2R.png')));
    elseif(videonumber >= 19 && videonumber <= 26 || videonumber == 31 || videonumber == 30 || videonumber == 22)
        maskBoatL = rgb2gray(imread(fullfile(boatMasksPath, 'maska3L.png')));
        maskBoatR = rgb2gray(imread(fullfile(boatMasksPath, 'maska3R.png')));
    end
    
    nObj_L = numel(objs_L);
    nObj_R = numel(objs_R);
    
    objects_L = [];
    objects_R = [];
    
    if(nObj_L == 0 && nObj_R == 0)
        objects_L = objs_L;
        objects_R = objs_R;
    elseif(nObj_L > 0 && nObj_R == 0)
        objects_R = objs_R;
        objects_L = filterBoatDetections(objs_L, nObj_L, maskBoatL);
    elseif(nObj_R > 0 && nObj_L == 0)
        objects_R = filterBoatDetections(objs_R, nObj_R, maskBoatR);
        objects_L = objs_L;
    elseif(nObj_R > 0 && nObj_R)
        objects_R = filterBoatDetections(objs_R, nObj_R, maskBoatR);
        objects_L = filterBoatDetections(objs_L, nObj_L, maskBoatL);
    end

end
    
function filtered_objects = filterBoatDetections(objs, n, mask)
    filtered_objects = [];
    stObj = 1;
    for i = 1 : n
        %check if detection overlaps boat part..
        if(boatMaskOverlap(mask, objs(i).bounding_box(1), objs(i).bounding_box(2), ...
                    objs(i).bounding_box(3), objs(i).bounding_box(4)) == 1)
                
                filtered_objects(stObj).bounding_box = objs(i).bounding_box;
                stObj = stObj + 1;
        end
    end

end

%if detection overlaps boat part return 0, else return 1...
function booleanOverlap = boatMaskOverlap(mask, xLeft, yTop, width, height)
    m1 = imcrop(mask, [xLeft, yTop, width, height]);
    numOnes = length(find(m1));
    numAll = width * height;
    if(numOnes >= 0.4 * numAll)
        booleanOverlap = 0;
    else
        booleanOverlap = 1;
    end
end