function EpiLines = getEpipolarLines(points, F, camera)
    % points - center points of obstacles in left camera
    % F - fundamental matrix
    % camera = 1 for left camera, 2 for right camera
    s = size(points);
    p = cell(s(1),1);
    for i = 1 : s(1)
        p{i} = points(i, :);
    end
    % Return epilines in format [a,b,c] where a*x + b*y + c = 0;
    %EpiLines = cv.computeCorrespondEpilines(points, F.FundMat, 'WhichImage', camera);
    EpiLinesCell = cv.computeCorrespondEpilines(p, F.FundMat, 'WhichImage', camera);
    EpiLines = zeros(numel(EpiLinesCell), 3);
    for i = 1 : numel(EpiLinesCell)
        EpiLines(i,:) = EpiLinesCell{i}';
    end
    
end