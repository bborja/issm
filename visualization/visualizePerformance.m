function visualizePerformance(dataset_path, vid_number, results, varargin)
    num_argin = numel(varargin);

    %Display switches...
    % 1 - horizon
    % 2 - sea edge
    % 3 - sea component
    % 4 - detections
    % 5 - ground truth obstacles
    % 6 - ground truth edge
    display_switches = zeros(6,1);
    for i = 1 : num_argin
       switch varargin{i}
           case 'hor'
               display_switches(1) = 1;
           case 'edg'
               display_switches(2) = 1;
           case 'sea'
               display_switches(3) = 1;
           case 'det'
               display_switches(4) = 1;
           case 'gtobs'
               display_switches(5) = 1;
           case 'gtedg'
               display_switches(6) = 1;
       end
    end
    
    [vid_seq_name, ~, start_frame, end_frame, ~, ~] = getSequenceDetails(vid_number);
	num_of_frames = end_frame - start_frame;
    
    % Root directories paths
	images_root_dir = fullfile(dataset_path, 'video_data', vid_seq_name, 'frames');
    gt_root_dir = fullfile(dataset_path, 'annotations', vid_seq_name, 'groundTruth');

    if(vid_number == 1)
        num_of_frames = 250;
    end
    figure(1);
    for i = 1 : num_of_frames
       img_l = imread(fullfile(images_root_dir, sprintf('%08dL.jpg', i + start_frame)));
       sim = size(img_l);
       gt_l = load(fullfile(gt_root_dir, sprintf('%08dL.mat', i + start_frame)));
       
       clf;
       %display image
       %if sea component switch is enabled, then display black and white
       %image with blue-colored sea component, else display rgb raw image
       if(display_switches(3) == 1)
            M2 = img_l ; Im2 = img_l;
            grIm = rgb2gray(img_l);
            maskedSea = imresize(results{vid_number, 1}.sea_edge{i,1}, [sim(1), sim(2)]); %imresize(maskedSea, [sizeY, sizeX]);
            for j = 1 : 3
                M2(:,:,j) = maskedSea;
                Im2(:,:,j) = grIm;
            end 

            clr = [0, 0, 200];
            w = [0.4, 0.4, 0];

            for j = 1 : 3
                grIm_b = double(grIm);
                grIm_b(maskedSea) = grIm_b(maskedSea)*w(j) + clr(j);    
                Im2(:,:,j) = grIm_b;
            end
            imagesc(Im2); axis equal; axis tight; axis off; hold on; title(sprintf('Frame: %d / %d', i, num_of_frames));
       else
            imagesc(img_l); axis equal; axis tight; axis off; hold on; title(sprintf('Frame: %d / %d', i, num_of_frames));
       end
       
       %display horizon
       if(display_switches(1) == 1)
           %todo. no horizon information saved in the output currently. use
           %other display function in order to display the estimated
           %horizon
       end
       if(display_switches(6) == 1)
          sea_edge = gt_l.sea_edge; 
          plot(sea_edge(:,1), sea_edge(:,2), 'LineWidth', 2, 'color', [1,0,1]); hold on;
       end
       
       
       %display sea edge estimation
       if(display_switches(2) == 1)
%            edge_list = extractSeaEdge(imresize(results{vid_number,1}.sea_edge{i,1}, [sim(1), sim(2)]));
%            valid_ind = find(edge_list(1,:) >= 1 & edge_list(1,:) <= sim(2) & edge_list(2,:) < sim(1));
% 
%            plot([1, edge_list(1,valid_ind), sim(2)], [sim(1), edge_list(2,valid_ind), sim(1)], 'LineWidth', 2, 'color', [0,0,1]); hold on;
            drawEdgeContour(imresize(results{vid_number, 1}.sea_edge{i, 1}, [sim(1), sim(2)]));
       end
       
       %display (ALL!) annotated ground truth obstacles
       %note that some of these obstacles are too small and are not used in
       %the process of evaluation. Also, smaller obstacles near the
       %sea-edge are removed from evaluation as well as larger obstacles
       %which pertrude the sea edge with its bounding box.
       if(display_switches(5) == 1)
           % Display ground truth small objects...
           for k = 1 : size(gt_l.smallobjects, 1)
               tmp_obj = gt_l.smallobjects(k,:);
               tmp_obj(3) = round(tmp_obj(3) - tmp_obj(1));
               tmp_obj(4) = round(tmp_obj(4) - tmp_obj(2));
               rectangle('Position', tmp_obj, 'LineWidth', 2, 'EdgeColor', 'g'); hold on;                
           end
           % Display ground truth large objects...
           for k = 1 : size(gt_l.largeobjects, 1)
               tmp_obj = gt_l.largeobjects(k,:);
               tmp_obj(3) = round(tmp_obj(3) - tmp_obj(1));
               tmp_obj(4) = round(tmp_obj(4) - tmp_obj(2));
               rectangle('Position', tmp_obj, 'LineWidth', 2, 'EdgeColor', 'w'); hold on;
           end
       end

       %display detected obstacles
       if(display_switches(4) == 1)
           detections = results{vid_number}.detections{i, 1};
           num_detections = numel(detections); %size(detections, 1);
           for j = 1 : num_detections
              rectangle('Position', detections(j).bounding_box, 'LineWidth', 3, 'EdgeColor', 'k'); hold on;
              rectangle('Position', detections(j).bounding_box, 'LineWidth', 2, 'EdgeColor', 'y'); hold on;
           end
       end
       
%        if(i == 50 || i == 205)
%            drawnow;
%        end
       drawnow;
    end
end

%Function to extract contour of sea edge based on obstacle binary mask
function edge_points = drawEdgeContour(masked_sea)

    %Fill against the right and bottom border
    bw_c = padarray(masked_sea,[1 1],1,'post');
    bw_c_filled = imfill(bw_c,'holes');
    bw_c_filled = bw_c_filled(1:end-1,1:end-1);

    %Fill against the bottom and left border
    bw_d = padarray(padarray(masked_sea, [1 0], 1, 'post'), [0 1], 1, 'pre');
    bw_d_filled = imfill(bw_d,'holes');
    bw_d_filled = bw_d_filled(1:end-1,2:end);
    
    %Combine fills
    bw_filled = bw_c_filled | bw_d_filled;
    
    [edge_points, ~] = contour(bw_filled, 'LineWidth', 2, 'color', [0,0,1]);
    
end