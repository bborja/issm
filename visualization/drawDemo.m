% Function to draw everything for demo:
%	+ raw images
%	+ sea edge estimation
%	+ generated conditional priors of semantic components
%	+ estimated horizon location
%	+ modified hiper-priors
%	+ all detected obstacles (black dotted bounding boxes)
%	+ detected obstacles (yellow bounding boxes)
function drawDemo(im_l, q_l, im_r, q_r, priors_sea_l, priors_sea_r, priors_sky_l, priors_sky_r, hor_line_param, original_objs_l, original_objs_r, detector_l, detector_r, gt_l, gt_r, avg_hor_centery_l, avg_hor_centery_r, hiper_priors_fix, hor_angle_l, hor_angle_r)
	sim = size(im_l);
	
	% Draw basic sea edge estimation and obstacles...
	figure(1); clf;
	subplot(2,2,1);
		drawBasic(im_l, detector_l, original_objs_l, gt_l); title('Left camera');
	subplot(2,2,2);
		drawBasic(im_r, detector_r, original_objs_r, gt_r); title('Right camera');
    subplot(2,2,3);
        imagesc(invert_q_mask(q_l(:,:,1:3))); axis equal; axis tight; title('Segmentation mask Left');
    subplot(2,2,4);
        imagesc(invert_q_mask(q_r(:,:,1:3))); axis equal; axis tight; title('Segmentation mask Right');
	

	% Draw horizon, hyper-priors and stuff
	figure(2); clf;
	subplot(2,3,1);
		drawHorizonHyper(im_l, hor_line_param(1:2), detector_l, sim, avg_hor_centery_l, hiper_priors_fix, hor_angle_l); title('Left camera');
	subplot(2,3,2);
		imagesc( imresize( priors_sea_l, sim(1:2) )); colormap gray; axis equal; axis tight; title('Sea component conditional prior');
	subplot(2,3,3);
		imagesc( imresize( priors_sky_l, sim(1:2) )); colormap gray; axis equal; axis tight; title('Sky component conditional prior');
	subplot(2,3,4);
		drawHorizonHyper(im_r, hor_line_param(3:4), detector_r, sim, avg_hor_centery_r, hiper_priors_fix, hor_angle_r); title('Right camera');
	subplot(2,3,5);
		imagesc( imresize( priors_sea_r, sim(1:2) )); colormap gray; axis equal; axis tight; title('Sea component conditional prior');
	subplot(2,3,6);
		imagesc( imresize( priors_sky_r, sim(1:2) )); colormap gray; axis equal; axis tight; title('Sky component conditional prior');

end


% Draw basic obstacle detection for demo presentation
function drawBasic(im, detector, orig_objs, gt)
	
	imagesc(im); axis equal; axis tight; hold on;
	% Draw estimated sea edge
	scatter(detector.sel_xy(1,:), detector.sel_xy(2,:)); hold on;
	% Draw all detections
	drawObjs(orig_objs, 'r');
	% Draw kept detections
	drawObjs(detector.objs, 'y');

end

function drawObjs(objects_list, det_color)
	num_objects = size(objects_list, 1);
	
	for i = 1 : num_objects
		rectangle('Position', round(objects_list(i).bounding_box), 'EdgeColor', 'k', 'LineWidth', 2); hold on;
		rectangle('Position', round(objects_list(i).bounding_box), 'EdgeColor', det_color, 'LineWidth', 1); hold on;
	end
end

function drawHorizonHyper(im, hor_line_param, detector, sim, avg_hor_centery, hiper_priors_fix, hor_angle)
	points_x_hor_plot = [1, sim(2)];
	points_y_hor_plot =  points_x_hor_plot * hor_line_param(1) + hor_line_param(2);
    scaled_image_size = [50, 50];
	
	%imagesc(im); axis equal; axis tight; hold on; (full resolution image)
    imagesc(imresize(im, scaled_image_size)); axis equal; axis tight; hold on; % image scalled to 50x50
	% Plot estimated horizon (for full resolution image only)
	%plot( points_x_hor_plot, points_y_hor_plot, 'k', 'LineWidth', 2); hold on;
	%plot( points_x_hor_plot, points_y_hor_plot, 'y', 'LineWidth', 1); hold on;
	
	% Plot modified hyper-priors (on 50x50 image)
    detector.detector_state.prior_mixture = changeHipers(detector, avg_hor_centery, hiper_priors_fix, hor_angle, scaled_image_size);
    
    % scale factors if we want to scale this visualization to full
    % resolution image...
    mu_scale = [sim(2); sim(1)] ./ [scaled_image_size(2); scaled_image_size(1)];
    cov_scale = [sim(2), 1; 1, sim(1)] ./ [scaled_image_size(2), 1; 1, scaled_image_size(1)]; 
    colors = [0,0,0; 1,0,0; 0,1,0; 0,0,1];
    for i = 1 : 3
        %plot_gaussian_ellipsoid(detector.detector_state.prior_mixture(i).Mu(1:2) .* mu_scale - ([25;0] .* mu_scale), detector.detector_state.prior_mixture(i).Cov(1:2,1:2) .* cov_scale, 3, colors(1,:)); hold on;
        %plot_gaussian_ellipsoid(detector.detector_state.prior_mixture(i).Mu(1:2) .* mu_scale - ([25;0] .* mu_scale), detector.detector_state.prior_mixture(i).Cov(1:2,1:2) .* cov_scale, 3, colors(1+i,:)); hold on;
        plot_gaussian_ellipsoid(detector.detector_state.prior_mixture(i).Mu(1:2) - [25; 0], detector.detector_state.prior_mixture(i).Cov(1:2,1:2), 3, colors(1,:)); hold on;
        plot_gaussian_ellipsoid(detector.detector_state.prior_mixture(i).Mu(1:2) - [25; 0], detector.detector_state.prior_mixture(i).Cov(1:2,1:2), 3, colors(1+i,:)); hold on;
    end
	
end


function prior_mixture = changeHipers(detector, avg_hor_centery, hiper_priors_fix, hor_angle, scaled_image_size)
    [tmp_mus, tmp_cov_landcomp] = improveHiperPriors([detector.detector_state.prior_mixture(1).Mu'; ...
												      detector.detector_state.prior_mixture(2).Mu'; ...
													  detector.detector_state.prior_mixture(3).Mu'], ...
												      detector.detector_state.prior_mixture(2).Cov, ...
													  avg_hor_centery, hiper_priors_fix, hor_angle, scaled_image_size);
            
    % update hiperpriors with new values
    for i = 1 : 3
        detector.detector_state.prior_mixture(i).Mu = tmp_mus(i,:)';
    end
    detector.detector_state.prior_mixture(2).Cov = tmp_cov_landcomp;
    prior_mixture = detector.detector_state.prior_mixture;
end

function new_q = invert_q_mask(q)
    [size_y, size_x, size_c] = size(q);
    new_q = zeros(size_y, size_x, size_c);
    for i = 1 : 3
        new_q(:,:,i) = q(:,:,i)';
    end
end