function [sea_edge, detections, times] = perform_detections_SSM(dataset_path, video_number, type_colorspace, scaled_image_size, stereo_ver)
    % Get video sequence details
    [vid_seq_name, calib_seq_name, start_frame, end_frame, ~, ~] = getSequenceDetails(video_number);
	num_of_frames = end_frame - start_frame;
		
	% Root directories paths
	images_root_dir = fullfile(dataset_path, 'video_data', vid_seq_name, 'frames');
    	 
	% Get Fundamental matrix for stereo verification if needed
    if( stereo_ver )
        F = load(fullfile(dataset_path, 'video_data', calib_seq_name, 'fundamentalMatrix.mat')); %we load fundamental matrix from file...
    end
	
	% Get image size if "fs" file was not loaded
	if(~exist('sim', 'var'))
		sim = size( imread(fullfile( images_root_dir, sprintf( '%08dL.jpg', start_frame ))));
	end
	 
	% Initialize detector state for left and right camera 
	detector_l.detector_state = DetectorState;
	detector_r.detector_state = DetectorState;
    
    fprintf('Processing video number %2d with options:\n\t*Stereo verification: %1d\n\t*Image size: %d x %d\n\t*Processing frame 1 / %d ...\n', ...
			video_number, stereo_ver, scaled_image_size(2), scaled_image_size(1), num_of_frames);
	
	% Timers initialization..
	time_segm = 0;
	time_stereo_ver = 0;
    
    % Obstacle detection measures
    sea_edge = cell(num_of_frames, 2);
	detections = cell(num_of_frames, 2);	
    times = cell(num_of_frames, 4);
	
    for fr = 1 : num_of_frames
        method_crashed = 0;
        
        if(mod(fr,30) == 0)
            fprintf('\t*Processing frame %d / %d ...\n', fr, num_of_frames);
        end

        % Load images
        im_l = imread( fullfile( images_root_dir, sprintf( '%08dL.jpg', fr + start_frame ) ) );
        im_r = imread( fullfile( images_root_dir, sprintf( '%08dR.jpg', fr + start_frame ) ) );

        try            
            % Set colorspaces
            detector_l.detector_state.colorspace = type_colorspace;
            detector_r.detector_state.colorspace = type_colorspace;


            % Perform ISSM_s on the images...
            detector_state_l = detector_l.detector_state;
            detector_state_r = detector_r.detector_state;

            
            tic;
            [detector_state_l, sel_xy_l, objs_l, masked_sea_l] = detect_edge_of_sea_simplified_SSM(detector_state_l, im_l, scaled_image_size);
            time_segm_l = toc;
            tic;
            [detector_state_r, sel_xy_r, objs_r, masked_sea_r] = detect_edge_of_sea_simplified_SSM(detector_state_r, im_r, scaled_image_size);
            time_segm_r = toc;
            %time_segm = toc;
            time_segm = time_segm_l + time_segm_r;
            
            detector_l.detector_state = detector_state_l;
            detector_r.detector_state = detector_state_r;

            detector_l.objs = objs_l;
            detector_r.objs = objs_r;

            detector_l.sel_xy = sel_xy_l;
            detector_r.sel_xy = sel_xy_r;

            detector_l.maskedSea = masked_sea_l;
            detector_r.maskedSea = masked_sea_r;

            detector_l.is_initialized = true;
            detector_r.is_initialized = true;


            % Discard detections that are due to fisheye camera vignette or parts of the USV visible in camera's field of view
            [tmp_objects_l, tmp_objects_r] = discardBoatDetections(dataset_path, detector_l.objs, detector_r.objs, sim, video_number);
            detector_l.objs = tmp_objects_l;
            detector_r.objs = tmp_objects_r;
            clear tmp_objects_l tmp_objects_r;

            % Perform stere verification of the obstacles if enabled...
            tic;
            if( stereo_ver )              
                [tmp_objects_l, tmp_objects_r] = verifyObstacleConsistency(im_l, im_r, detector_l.objs, detector_r.objs, F);               
                detector_l.objs = tmp_objects_l;
                detector_r.objs = tmp_objects_r;
                clear tmp_objects_l tmp_objects_r;

            end
            time_stereo_ver = toc;			

        catch err % Catch if any error occured (this shouldnt happend)
            fprintf('Method crashed on frame %04d\n', fr);
            errMsgTxt = getReport(err);
            fprintf('%s\n', errMsgTxt);
            method_crashed = 1;
        end

        if(method_crashed == 0)
           sea_edge{fr,1} = detector_l.maskedSea;
           sea_edge{fr,2} = detector_r.maskedSea;

           detections{fr,1} = detector_l.objs;
           detections{fr,2} = detector_r.objs;
           
           times{fr,1} = time_segm;
           times{fr,2} = time_stereo_ver;
           times{fr,3} = time_segm_l;
           times{fr,4} = time_segm_r;
           
        else
            sea_edge{fr,1} = [];
            sea_edge{fr,2} = [];

            detections{fr,1} = [];
            detections{fr,2} = [];
            
            times{fr,1} = 0;
            times{fr,2} = 0;
            times{fr,3} = 0;
            times{fr,4} = 0;
        end
    end
end