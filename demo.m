% Demo function for presentation of our proposed method...
% Draws sea edge estimation and all detections on video
%	along with some additional stuff...
% Input: Relative path to our dataset
function demo(dataset_path)
	add_all_paths;
    warning('off', 'all');

	% demo video sequence...
	% Note (you can change this to any number from interval [1,28]
	%test_vid_number = 18;
    test_vid_number = 7;
	type_colorspace = Colorspace.rgb;
	
	
	% Get video sequence details: - name of the sequence
	%							  - name of the corresponding calibration sequence
	%							  - start frame number
	%							  - end frame number
	[vid_seq_name, calib_seq_name, start_frame, end_frame, precalib_offset_one, precalib_offset_two] = getSequenceDetails(test_vid_number);
	num_of_frames = end_frame - start_frame;
	
	
	% Root directories paths
	%images_root_dir = fullfile(dataset_path, 'video_data', vid_seq_name, 'frames');
    images_root_dir = fullfile(dataset_path, 'video_data', vid_seq_name, 'framesRectified');
    gt_root_dir = fullfile(dataset_path, 'annotations', vid_seq_name, 'groundTruth');
	
	
	% Method functions: - imu_sensor (should the method use the IMU information?)
	%				    - stereo_ver (should the method use stereo verification?)
	%					- scaled_image_size (segmentation image size in pixels)
	%					- num_cond_priors (which conditional priors to generate?)
	imu_sensor = 1; % 0 = off, 1 = on
	stereo_ver = 1; % 0 = off, 1 = on %1 orig
	%scaled_image_size = [50, 50]; %orig
	scaled_image_size = [100, 100]; 
    num_cond_priors = 1; % 1 = sky and sea, 2 = sky only, 3 = sea only
	
	
	% Load IMU sensor data if needed
	if( imu_sensor )
		% Load hiperpriors information
		hiper_priors_fix = load('hiperprior-data/avgHorY.mat');       
	
		%load(fullfile(precalib_path, 'points.mat'));
		%load(fullfile(precalib_path, 'imu1.mat'));
		%load(fullfile(precalib_path, 'imu2.mat'));
        fs = cv.FileStorage(fullfile(dataset_path, 'video_data', vid_seq_name, 'calibration.yaml'));

        % Load imu calibration
        % The IMU data format changes after sequence 15, because of the
        % on-board software update...
        if(test_vid_number < 15)
            [B, ~, imu_initial_pitch, imu_initial_roll] = getGroundPlane( fullfile(dataset_path, 'video_data', calib_seq_name), 100);
        else
            [B, ~, imu_initial_roll, imu_initial_pitch] = getGroundPlane( fullfile(dataset_path, 'video_data', calib_seq_name), 100);
        end
        
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
	
	
	fprintf('Processing video number %2d with options:\n\t*IMU sensor: %1d\n\t*Stereo verification: %1d\n\t*Image size: %d x %d\n', ...
			test_vid_number, imu_sensor, stereo_ver, scaled_image_size(2), scaled_image_size(1));
			
	
	% Timers initialization..
	time_segm = 0;
	time_stereo_ver = 0;
		
	
	for fr = 1 : num_of_frames
		% Load images
		im_l = imread( fullfile( images_root_dir, sprintf( '%08dL.jpg', fr + start_frame ) ) );
        im_r = imread( fullfile( images_root_dir, sprintf( '%08dR.jpg', fr + start_frame ) ) );
        
        % Load ground truth files...
        gt_l = load(fullfile(gt_root_dir, sprintf('%08dL.mat', fr + start_frame)));
        %gt_l = gt_l.new_gt_l;
        % load ground truth file for right camera in case of simple/mono
        % segmentation
        gt_r = load(fullfile(gt_root_dir, sprintf('%08dR.mat', fr + start_frame)));
        %gt_r = gt_r.new_gt_r;

        
        try            
            % Set colorspaces
            detector_l.detector_state.colorspace = type_colorspace;
            detector_r.detector_state.colorspace = type_colorspace;
			
			% Build conditional priors if the IMU sensor is used
            if( imu_sensor )
				% Get horizon parameters...
				[hor_line_param, ~] = getVanishingPoints(B, imu_initial_pitch, imu_initial_roll, fr + start_frame, fullfile(dataset_path, 'video_data', vid_seq_name), fs, test_vid_number, precalib_offset_one, precalib_offset_two);

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
			
			tic;
			%detector_l = water_perform_detection_on_image_ISSM(detector_l, im_l, 6.15, scaled_image_size, priors_sea_l, priors_sky_l, hor_angle_l, avg_hor_centery_l, hiper_priors_fix);
			%detector_r = water_perform_detection_on_image_ISSM(detector_r, im_r, 6.15, scaled_image_size, priors_sea_r, priors_sky_r, hor_angle_r, avg_hor_centery_r, hiper_priors_fix);
			[detector_state_l, sel_xy_l, objs_l, masked_sea_l, q_sum_large_l] = detect_edge_of_sea_simplified_ISSM(detector_state_l, im_l, 6.15, scaled_image_size, 'prior_sea', priors_sea_l, 'prior_sky', priors_sky_l, 'avg_hor_y', avg_hor_centery_l, 'hor_angle', hor_angle_l, 'hiper_fix', hiper_priors_fix);
			[detector_state_r, sel_xy_r, objs_r, masked_sea_r, q_sum_large_r] = detect_edge_of_sea_simplified_ISSM(detector_state_r, im_r, 6.15, scaled_image_size, 'prior_sea', priors_sea_r, 'prior_sky', priors_sky_r, 'avg_hor_y', avg_hor_centery_r, 'hor_angle', hor_angle_r, 'hiper_fix', hiper_priors_fix);
			time_segm_single = toc;
            time_segm = time_segm + time_segm_single;
			
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
            [tmp_objects_l, tmp_objects_r] = discardBoatDetections(dataset_path, detector_l.objs, detector_r.objs, sim, test_vid_number);
            detector_l.objs = tmp_objects_l;
            detector_r.objs = tmp_objects_r;
			clear tmp_objects_l tmp_objects_r;
			
			
			% Only used later on for visualization of obstacles discarded by the stereo verification method...
			original_objs_l = detector_l.objs;
			original_objs_r = detector_r.objs;
			
			
			% Perform stere verification of the obstacles if enabled...
			tic;
            if( stereo_ver )              
                [tmp_objects_l, tmp_objects_r] = verifyObstacleConsistency(im_l, im_r, detector_l.objs, detector_r.objs, F);               
                detector_l.objs = tmp_objects_l;
                detector_r.objs = tmp_objects_r;
                clear tmp_objects_l tmp_objects_r;

            end
            time_stereo_ver_single = toc;
			time_stereo_ver = time_stereo_ver + time_stereo_ver_single;
			
		
			% Draw detection and water edge estimation...
			drawDemo(im_l, q_sum_large_l, im_r, q_sum_large_r, priors_sea_l, priors_sea_r, priors_sky_l, priors_sky_r, hor_line_param, original_objs_l, original_objs_r, detector_l, detector_r, gt_l, gt_r, avg_hor_centery_l, avg_hor_centery_r, hiper_priors_fix, hor_angle_l, hor_angle_r); drawnow;
			fprintf('Frame %03d done. Time for segmentation %0.3fs, time for stereo verification %0.3fs\n', fr, time_segm_single, time_stereo_ver_single);
			
			
		catch err % Catch if any error occured (this shouldnt happend)
            fprintf('Method crashed on frame %04d\n', fr);
            errMsgTxt = getReport(err);
            fprintf('%s\n', errMsgTxt);
        end
	end
	
	fprintf('Demo finished....\n\tAverage time for segmentation: %1.3f\n\tAverage time for stereo verification: %1.3f\n\tFrame rate: %2.1f\n', (time_segm / num_of_frames), (time_stereo_ver / num_of_frames), 1 / ( (time_segm / num_of_frames) + (time_stereo_ver / num_of_frames) ) ); 
end