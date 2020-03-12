function [ detector_state, sel_xy, objects, masked_sea ] = detect_edge_of_sea_simplified_SSM(detector_state, im, scaled_image_size)
   
	
	% Initialize detector on first pass
    initialize_detector = ~detector_state.is_initialized;
    if initialize_detector,
        % Create detector state
        detector_state.is_initialized = true;     
        
        % Load prior
        [ model_prior, em_image_size ] = loadPriorModelFromDisk(detector_state.colorspace);
        if(~isequal(em_image_size, scaled_image_size))
           model_prior = scaleModelPriors(model_prior, em_image_size, scaled_image_size);
           em_image_size = scaled_image_size;
        end

%        assert(isequal(em_image_size, scaled_image_size), 'Change this if EM image size is not equal to specified size');
        
		detector_state.prior_mixture = model_prior.mixture;
        % Set EM image size (stored with prior model)
        detector_state.em_image_size = em_image_size;

		
        % Initialize spatial data - this in fact describes position of each
        % pixel... we use ndgrid instead of meshgrid because it returns correct
        % shape of the array
        [ x, y ] = ndgrid(1:em_image_size(2), 1:em_image_size(1));
        detector_state.spatial_data = [ x(:)'; y(:)' ];
    end


    %% Prepare data for EM
    % If first dimension of the image appear to be channels, assume
    % OpenCV-compatible layout: interleaved BGR in row-major order. Otherwise,
    % assume we have Matlab-compatible layout
    if size(im, 1) == 3,
        % Deinterleave image, transpose and switch BGR to RGB
        B = squeeze(im(1:3:end,:,:));
        G = squeeze(im(2:3:end,:,:));
        R = squeeze(im(3:3:end,:,:));

        tmp_im = zeros(size(im, 3), size(im, 2), size(im, 1), 'uint8');

        tmp_im(:,:,1) = R(:,:,1)';
        tmp_im(:,:,2) = G(:,:,1)';
        tmp_im(:,:,3) = B(:,:,1)';

        image_transposed = true;

    else
        tmp_im = im;
        image_transposed = false;

    end

    % Scale the image 
    Ax1 = imresize(tmp_im, detector_state.em_image_size, 'bilinear');

    % Convert color space
    switch detector_state.colorspace,
        case Colorspace.hsv, 
              Ax = rgb2hsv(Ax1);
        case Colorspace.rgb,
            Ax = double(Ax1);
        case Colorspace.ycrcb, 
            Ax = double( rgb2ycbcr(Ax1));

        case Colorspace.lab, 
            Ax = rgb2lab(Ax1) ;
        case Colorspace.ycrs, 
            Ahsv =  rgb2hsv(Ax1) ;
            Ax = double(rgb2ycbcr(Ax1));
            Ax(:,:,3) = Ahsv(:,:,3) ;
        otherwise
            error('Unknown colorspace.');
    end

    % Construct the color data - transpose Matlab (column-major order, 
    % deinterleaved RGB) image via permute, so that we get C-ordering
    % (row-major) of data, then unroll image planes into vectors
    color_data = reshape( permute(Ax, [ 2 1 3 ]), [], 3 )';

    dataEM = [ detector_state.spatial_data; color_data ];

    sim	= size(tmp_im);
    sim = sim([1 2]) ;

    %% initialize the EM if no prior mixture exists
    if initialize_detector,
        df = [0, 0.3, 0.5 , 1] ;
        vertical_ratio = df*size(dataEM,2);

        for k = 1 : 3
            dat = dataEM(:,vertical_ratio(k)+1:vertical_ratio(k+1)) ;
			
            detector_state.current_mixture(k).Mu = mean(dat,2) ;
            detector_state.current_mixture(k).Cov = cov(dat')  ;
            %detector_state.current_mixture(k).Cov = modify_cov_matrix(detector_state.current_mixture(k).Cov, rotAngl); %rotate covariance
            detector_state.current_mixture(k).w = 1/3;
        end
    else
        if ~isequal(detector_state.colorspace, Colorspace.hsv)
            % mix existing initial distribution with the current initial pdf

            df = [0 0.2; 0.2 0.4; 0.6 1] ;
            vertical_ratio = df*size(dataEM, 2) ;

            w_init = 0.4 ;
            w_mix = [1-w_init, w_init] ;

            for k = 1 : 3

				dat = dataEM(:,vertical_ratio(k,1)+1:vertical_ratio(k,2)) ;
                
                tmp_mu = mean(dat,2);
                tmp_cov = cov(dat');
                
                detector_state.current_mixture(k) = momentMatchPdf(detector_state.current_mixture(k).Mu,tmp_mu, ...
																   detector_state.current_mixture(k).Cov, tmp_cov , w_mix) ;
            end

            sum_w = 0;
			
            for k=1:3,
                sum_w=sum_w + detector_state.current_mixture(k).w;

            end
            
			for k=1:3,  
                detector_state.current_mixture(k).w = detector_state.current_mixture(k).w/ sum_w ;
            
			end
        end
    end
    

    [detector_state, Q_sum_large] = run_ISSM(detector_state, dataEM);



    %% Retrieve objects and sea edge
    [ objects, ~, sel_xy, masked_sea ] = getEdgeAndObjectsNoScaling(Q_sum_large, sim);

    % If we had to transpose the input image, we need to transpose masked_sea
    % and uncertainty_measures.uncertainty_map
    if image_transposed
        masked_sea = masked_sea'; 
    end
end

function new_model_prior = scaleModelPriors(model_prior, src_img_size, scaled_img_size)
    new_model_prior = model_prior;
    
    for k = 1 : 3
        new_model_prior.mixture(k).Mu(1) = scaled_img_size(1) / 2;
        new_model_prior.mixture(k).Mu(2) = model_prior.mixture(k).Mu(2) / src_img_size(2) * scaled_img_size(2);
        
        new_model_prior.mixture(k).Cov(1,1) = model_prior.mixture(k).Cov(1,1) / src_img_size(1) * scaled_img_size(1);
        new_model_prior.mixture(k).Cov(2,2) = model_prior.mixture(k).Cov(2,2) / src_img_size(2) * scaled_img_size(2);
    end
    
end

%--------------------------------------------------------------------------
function [ detector_state, Q_sum_large ] = run_ISSM(detector_state, data)

    % read the em parameters
    p_unknown = getUnknownWeightForTheFeatureModel(detector_state.colorspace, detector_state.em_image_size, ...
												   detector_state.use_uniform_component);

    maxEMsteps = detector_state.em_max_steps; % maximum number of em steps
    mask_size = detector_state.em_image_size;

    % construct convolution kernels for the em posteriors
    [H_0, H_1] = getConvolutionKernel(mask_size) ;

    % number of components in the mixture model
    mixture_size = length(detector_state.current_mixture) ;

    % intialize mixtures
    if isempty(detector_state.PI_i)
        PI_i = ones(mask_size(2), mask_size(1), mixture_size+1) /(mixture_size+1) ;
        PI_i(:,:,1:3) = 1/mixture_size - p_unknown/mixture_size ;
        PI_i(:,:,4) = p_unknown ;
    else
        PI_i = detector_state.PI_i ;
    end

    % initialize the posterior for checking stopping condition
    PI_i0 = PI_i ;% dbug

    cont = maxEMsteps ; % 100 ;

    Q_sum=[];
	
    p = zeros(mixture_size+1,size(data,2)) ;
	
     while cont > 1
        % get responsibilities
        for i = 1 : mixture_size
            p(i,:) = normpdf(data,detector_state.current_mixture(i).Mu,[], detector_state.current_mixture(i).Cov);
        end
        p(mixture_size+1,:) = p_unknown ;

        % for the paper:
        P_i = PI_i.*reshape(p', mask_size(2),mask_size(1), mixture_size+1) + eps;
        if ~detector_state.use_uniform_component,
            P_i(:,:,mixture_size+1) = 0 ;
        end

        % perform Expectation on prior and posterior
        P_i = bsxfun(@times, P_i, 1./sum(P_i,3)) ; 
        S_i = PI_i.*imfilter(PI_i , H_0, 'replicate')  ; S_i = bsxfun(@times, S_i, 1./sum(S_i,3)) ;
        
        Q_i = P_i.*imfilter(P_i , H_0, 'replicate')  ; Q_i = bsxfun(@times, Q_i, 1./sum(Q_i,3)) ;
        
        Q_sum = imfilter(Q_i , H_1, 'replicate') ;
        S_sum = imfilter(S_i , H_1, 'replicate') ;
        PI_i = (Q_sum + S_sum)*0.25 ; %
        

        % if not using the uniform component, set to zero
        if ~detector_state.use_uniform_component,
            PI_i(:,:,mixture_size+1) = 0 ;
        end

        p = reshape(Q_sum,mask_size(2)*mask_size(1),mixture_size+1)' ;

        % measure the change in posterior distribution
        d_pi = sum((sqrt(PI_i0)-sqrt(PI_i)),3) ;
        d_pi = sort(d_pi(:)) ;  Loglik_new = mean(d_pi(round(length(d_pi))/2:end)) ;
        if Loglik_new > 0.01 
            PI_i0 = PI_i ;% dbug
        else
            break ;
        end
        lp = sum(p,1) ;

        % normalize posterior
        p = bsxfun(@rdivide, p, lp) ;
        alpha_i = reshape(sum(sum(PI_i,1),2), 1, mixture_size+1 ) ;

        % Maximization on Guassian parameters
        alpha_i = alpha_i / sum(alpha_i) ;
        % estimate means and covs 
        for k = 1 : mixture_size  
            w_data = bsxfun(@times, data, p(k,:)) ;
            x_mu = sum( w_data, 2 ) / sum(p(k,:)) ;
            x_2_mu = data*w_data' / sum(p(k,:)) ;                
            c = x_2_mu - x_mu*x_mu' ;                
            % naiive update of mean and covariance      
            detector_state.current_mixture(k).Cov = c;
            detector_state.current_mixture(k).Mu = x_mu ;
            detector_state.current_mixture(k).w = alpha_i(k) ;

            % update means with a static prior
            if detector_state.use_prior_on_mixture
                i_c_d = k ; i_c_0 = k ;
                detector_state.current_mixture(i_c_d).Mu = mergePd( detector_state.current_mixture(i_c_d).Mu, ...
                    detector_state.current_mixture(i_c_d).Cov , ...
                    detector_state.prior_mixture(i_c_0).Mu,...
                    detector_state.prior_mixture(i_c_0).Prec) ; 
            end
        end     
        sum_w=0;
        for i=1:numel(detector_state.current_mixture),
            sum_w=sum_w+detector_state.current_mixture(i).w;
        end;

        for k=1:mixture_size,
            detector_state.current_mixture(k).w = detector_state.current_mixture(k).w/ sum_w ;
        end

         cont = cont - 1 ;
     end
     
    Q_sum_large = Q_sum ;
    detector_state.PI_i = Q_sum ; 
end
  
%-------------------------------------------------------------------------
function y = normpdf(x,mu,~,sigma)  
    [U,S,~] = svd(sigma) ;
    
    s = diag(S) ; s(s<eps) = 1 ; S = diag(s) ;
    A = bsxfun(@minus, x, mu)' * (U*diag(1./diag(sqrt(S)))) ;
    
    y = exp(-0.5*sum(A .* A, 2)) / (sqrt(2*pi) .* prod(diag(S))) ;
end
%-------------------------------------------------------------------------

function Mu = mergePd( Mu_d, C_d, Mu_0, iC0 )
     %#codegen 

    d = size(C_d,1) ;
    % approximately robust
    scl = 1e-10; 
    tmpB=diag(C_d);
    tmpC= C_d+eye(d)*mean(tmpB(:))*scl;

    Mu = (inv(tmpC) + iC0)\(tmpC\Mu_d + iC0*Mu_0) ;
end
 
%--------------------------------------------------------------------------
function [H_0, H_1] = getConvolutionKernel (sizeMask)
    %#codegen

    scale_filter = 0.1 ;

    %to je zaradi tega, ker coder ne podpira variablinih argumentov pri
    %fspecial
    %assert(isequal(sizeMask,[50,50]),'Slika mora biti 50x50.')

    hsize = ceil(0.2*( sizeMask(2)*scale_filter -1)) ;
    % H_0 = replacement_fspecial('gaussian', hsize*2+1, hsize/1.5) ;
    H_0 = fspecial('gaussian', hsize*2+1, hsize/1.5) ;

    centr = (size(H_0,1)-1)/2 + 1 ;
    H_0(centr, centr) = 0 ; H_0 = H_0 / sum(H_0(:)) ;
    H_1 = H_0 ;
    H_1(centr, centr) = 1 ;

end
 
%--------------------------------------------------------------------------

function p_unknown = getUnknownWeightForTheFeatureModel(type_colorspace, sizeMask, use_uniform_component) 
     %#codegen

    p_unknown=0;  %zato da je output predvidljiv

    if ~use_uniform_component
        return ;
    end

    switch(type_colorspace)
        case Colorspace.hsv
            p_unknown = 1/prod(sizeMask) ; 1e-5 ; 0.001 ;
        case Colorspace.rgb
            p_unknown = 1/(prod(sizeMask) *255^3) *0.001; 1e-5 ; 0.001 ;
        case Colorspace.ycrcb
            p_unknown = 0.001*1/(prod(sizeMask) *10988544) ; %
        case Colorspace.lab
            p_unknown =10^(-2) *  1 / ( prod(sizeMask) *255^3  ) ;%
        case Colorspace.ycrs
            p_unknown = 0.01*1/(prod(sizeMask) *49056) ; 
        otherwise
            error('Invalid type colorspace.');  
    end
end

%--------------------------------------------------------------------------
function [ objs_out, pts, xy_subset, masked_Sea ] = getEdgeAndObjectsNoScaling (P_edge , Im_size)
    %#codegen

    size_edge = Im_size ;
    size_obj = size(P_edge(:,:,3)) ;
    size_obj = size_obj([2,1]) ;
    scl = size_edge./size_obj ;
    Tt = diag(scl([2,1])) ;

    %% *** Sea edge and its uncertainty score ***
    [~,T] = max(P_edge,[],3) ; T = (T==3) ;

    T = bwmorph(T,'diag') ;
    T = extractTheLargestRegion(T) ;
    T = ~bwmorph(~T,'clean') ;
    masked_Sea = T' ;

    dT = [ zeros(size(T,1),1), diff(T')' ] ~=0 ;
    dT2 = [ zeros(1,size(T,2)); diff(T) ] ~=0 ;
    dT = dT | dT2 ;

    % try
    if (sum(T(:)) ~= numel(T)) && (sum(dT(:)) ~= 0),
        [ largest_curve, ~ ] = extractTheLargestCurve(dT') ;
        data = bsxfun(@minus, Tt*largest_curve', diag(Tt/2)) ;
    else
        objs_out = Object(0);
        pts = [ 0 0 0 0 ];
        xy_subset = [] ;
        return;
    end


    %% *** Edge of sea ***
    delta = size(P_edge,1)*0.3 ;
    [a, ~] = getOptimalLineImage_constrained(data, [], data, delta) ;
    xy_subset = data ;
        
    x0 = 0;
    y0 =  -(a(1)*x0 + a(3))/a(2);
    x1 = Im_size(2) ;
    y1 = -(a(1)*x1 + a(3))/a(2);

    pts = [ x0, y0, x1, y1 ];
    
	I = ~T';
    
    %% *** Detected objects ***
    CC = replacement_bwconncomp(I,8) ;

    % These are in fact "sub-boxes" which we merge into acutal object
    % detections
    objs = ObjectPart(numel(CC));

    counter = 0;
    for k = 1:numel(CC),
        pixels = CC(k).pixel_idx;

        obj = ObjectPart(1);

        %% Bounding box
        [ y, x ] = ind2sub(size_obj, pixels);
        xmin = min(x(:));
        ymin = min(y(:));
        xmax = max(x(:));
        ymax = max(y(:));

        xmin = xmin-1;
        ymin = ymin-1;
        width = xmax-xmin;
        height = ymax-ymin;

        % Rescale bounding box
        xmin = xmin*Tt(1);
        ymin = ymin*Tt(4);
        width = width*Tt(1);
        height = height*Tt(4);

        obj.bounding_box = [xmin, ymin, width, height];

        % Area
        obj.area = width*height;

        % Boundary
        [ymin,loc]=min(y(:));
        xmin = x(loc)*Tt(1);
        ymin = Tt(4)*ymin;


		[~,loc] = min(abs(xy_subset(1,:)-xmin)) ;
		boundary = xy_subset(:,loc) ;
        
		
        if boundary(2) > ymin
            continue;
        end

        %% Add to the list
        objs(counter+1) = obj;
        counter = counter+1;
    end

    objs(counter+1:end) = []; 

    objs_out = suppressDetections(objs);
end


function filtered_points = checkNearEdges(points, threshold_x, threshold_y)
    %threshold x and y in pixels....
    filtered_points = points;
    
    if((filtered_points(1,end) - filtered_points(1,end-1)) > threshold_x && abs(filtered_points(2,end)-filtered_points(2,end-1)) > threshold_y)
        filtered_points(2,end) = filtered_points(2, end-1); 
    end
    
    if((filtered_points(1,2) - filtered_points(1,1)) > threshold_x && abs(filtered_points(2,2) - filtered_points(2,1)) > threshold_y)
        filtered_points(2,1) = filtered_points(2,2);
    end
end
 



