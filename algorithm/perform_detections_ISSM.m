function [sea_edge, detections, times] = perform_detections_ISSM(dataset_path, video_number, type_colorspace, scaled_image_size, imu_sensor, num_cond_priors, stereo_ver)
    % Get video sequence details
    [vid_seq_name, calib_seq_name, start_frame, end_frame, precalib_offset_one, precalib_offset_two] = getSequenceDetails(video_number);
	num_of_frames = end_frame - start_frame;
		
	% Root directories paths
	images_root_dir = fullfile(dataset_path, 'video_data', vid_seq_name, 'frames');
    
    % Load IMU sensor data if needed
	if( imu_sensor )
		% Load hiperpriors information
		hiper_priors_fix = load('hiperprior-data/avgHorY.mat');
		% Path to precalibrated data of IMU sensor
		precalib_path = fullfile(dataset_path, 'annotations', vid_seq_name, 'IMUsensorCalib');
        
	
		load(fullfile(precalib_path, 'points.mat'));
		load(fullfile(precalib_path, 'imu1.mat'));
		load(fullfile(precalib_path, 'imu2.mat'));
		load(fullfile(precalib_path, 'fs.mat'));
		% Load imu calibration
		[B, ~, imu1, imu2] = getGroundPlane( fullfile(dataset_path, 'video_data', calib_seq_name), 100);
		fprintf('IMU calibration loaded\n');
		fprintf('IMU calibration files loaded...\n');
             
        sim = [fs.imageSize{2}, fs.imageSize{1}];
	 end
	 
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
    
    fprintf('Processing video number %2d with options:\n\t*IMU sensor: %1d\n\t*Stereo verification: %1d\n\t*Image size: %d x %d\n\t*Processing frame 1 / %d ...\n', ...
			video_number, imu_sensor, stereo_ver, scaled_image_size(2), scaled_image_size(1), num_of_frames);
	
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

            % Build conditional priors if the IMU sensor is used
            if( imu_sensor )
                % Get horizon parameters...
                [hor_line_param, ~] = getVanishingPoints(B, imu1, imu2, fr + start_frame, fullfile(dataset_path, 'video_data', vid_seq_name), fs, precalib_offset_one, precalib_offset_two);

                %debuging - horizon line
                %figure(23); clf; imshow(im_rec_l); hold on; plot([0,1500], horizon_params(1)*[0,1500] + horizon_params(2), 'linewidth', 3); drawnow;

                % Get Horizon angle
                hor_angle_l = atan(hor_line_param(1));
                hor_angle_r = atan(hor_line_param(3));

                % Get binary masks
                [priors_sea_l, priors_sea_r, priors_sky_l, priors_sky_r] = imuMask(hor_line_param, scaled_image_size, sim, 3);


                % Get y-position of horizon line's center
                avg_hor_centery_l = ( round( sim(2) / 2 ) * hor_line_param(1) + hor_line_param(2) ) / sim(1);
                avg_hor_centery_r = ( round( sim(2) / 2 ) * hor_line_param(3) + hor_line_param(4) ) / sim(1);

            end

            % Perform ISSM_s on the images...
            detector_state_l = detector_l.detector_state;
            detector_state_r = detector_r.detector_state;

            if(num_cond_priors == 0)
                tic;
                [detector_state_l, sel_xy_l, objs_l, masked_sea_l] = detect_edge_of_sea_simplified_ISSM(detector_state_l, im_l, 8, scaled_image_size, 'prior_sea', priors_sea_l, 'prior_sky', priors_sky_l, 'avg_hor_y', avg_hor_centery_l, 'hor_angle', hor_angle_l, 'hiper_fix', hiper_priors_fix);
                time_segm_l = toc;
                tic;
                [detector_state_r, sel_xy_r, objs_r, masked_sea_r] = detect_edge_of_sea_simplified_ISSM(detector_state_r, im_r, 8, scaled_image_size, 'prior_sea', priors_sea_r, 'prior_sky', priors_sky_r, 'avg_hor_y', avg_hor_centery_r, 'hor_angle', hor_angle_r, 'hiper_fix', hiper_priors_fix);
                time_segm_r = toc;
                time_segm = time_segm_l + time_segm_r;
                %time_segm = toc;
             
            elseif(num_cond_priors == 1)
                tic;
                [detector_state_l, sel_xy_l, objs_l, masked_sea_l] = detect_edge_of_sea_simplified_ISSM(detector_state_l, im_l, 8, scaled_image_size, 'prior_sea', priors_sea_l, 'prior_sky', priors_sky_l);
                time_segm_l = toc;
                tic;
                [detector_state_r, sel_xy_r, objs_r, masked_sea_r] = detect_edge_of_sea_simplified_ISSM(detector_state_r, im_r, 8, scaled_image_size, 'prior_sea', priors_sea_r, 'prior_sky', priors_sky_r);
                time_segm_r = toc;
                time_segm = time_segm_l + time_segm_r;
                %time_segm = toc;
                
            elseif(num_cond_priors == 2)
                tic;
                [detector_state_l, sel_xy_l, objs_l, masked_sea_l] = detect_edge_of_sea_simplified_ISSM(detector_state_l, im_l, 6.15, scaled_image_size, 'prior_sky', priors_sky_l);
                time_segm_l = toc;
                tic;
                [detector_state_r, sel_xy_r, objs_r, masked_sea_r] = detect_edge_of_sea_simplified_ISSM(detector_state_r, im_r, 6.15, scaled_image_size, 'prior_sky', priors_sky_r);
                time_segm_r = toc;
                time_segm = time_segm_l + time_segm_r;
                %time_segm = toc;
                
            elseif(num_cond_priors == 3)
                tic; 
                [detector_state_l, sel_xy_l, objs_l, masked_sea_l] = detect_edge_of_sea_simplified_ISSM(detector_state_l, im_l, 6.15, scaled_image_size, 'prior_sea', priors_sea_l);
                time_segm_l = toc;
                tic;
                [detector_state_r, sel_xy_r, objs_r, masked_sea_r] = detect_edge_of_sea_simplified_ISSM(detector_state_r, im_r, 6.15, scaled_image_size, 'prior_sea', priors_sea_r);
                time_segm_r = toc;
                time_segm = time_segm_l + time_segm_r;
                %time_segm = toc;
                
            elseif(num_cond_priors == 4)
                tic;
                [detector_state_l, sel_xy_l, objs_l, masked_sea_l] = detect_edge_of_sea_simplified_ISSM(detector_state_l, im_l, 6.15, scaled_image_size, 'avg_hor_y', avg_hor_centery_l, 'hor_angle', hor_angle_l, 'hiper_fix', hiper_priors_fix);
                time_segm_l = toc;
                tic;
                [detector_state_r, sel_xy_r, objs_r, masked_sea_r] = detect_edge_of_sea_simplified_ISSM(detector_state_r, im_r, 6.15, scaled_image_size, 'avg_hor_y', avg_hor_centery_r, 'hor_angle', hor_angle_r, 'hiper_fix', hiper_priors_fix);
                time_segm_r = toc;
                time_segm = time_segm_l + time_segm_r;
                %time_segm = toc;
                
            end

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