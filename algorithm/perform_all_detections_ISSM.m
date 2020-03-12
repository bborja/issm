function perform_all_detections_ISSM(dataset_path, original_ssm)
    add_all_paths;
    export_path = fullfile('output_detections');

	videos = [1:28];
	type_colorspace = Colorspace.rgb;
    scaled_image_size = [100,100];
    imu_sensor = 1;
    num_cond_priors = 1;
    %stereo_ver = 1;
    stereo_ver = 0;
	
    output_detections = cell(length(videos), 1);
	
    for i = 1 : length(videos)
        if(original_ssm)
            [sea_edge, detections, times] = perform_detections_SSM(dataset_path, videos(i), type_colorspace, scaled_image_size, stereo_ver);
        else
            [sea_edge, detections, times] = perform_detections_ISSM(dataset_path, videos(i), type_colorspace, scaled_image_size, imu_sensor, num_cond_priors, stereo_ver);
        end
        output_detections{i}.sea_edge = sea_edge;
        output_detections{i}.detections = detections;
        output_detections{i}.times = times;
    end
    
    if(original_ssm)
        file_name = sprintf('SSM_%dx%d_stereo-%d.mat', scaled_image_size(2), scaled_image_size(1), stereo_ver);
    else
        file_name = sprintf('ISSM_%dx%d_imu-%d_stereo-%d_condprior-%d_offsets0_stimes.mat', scaled_image_size(2), scaled_image_size(1), imu_sensor, stereo_ver, num_cond_priors);
    end

    save(fullfile(export_path, file_name), 'output_detections');
end