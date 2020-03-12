function [ model_prior, em_image_size ] = loadPriorModelFromDisk (type_colorspace)
%#codegen

model_prior = ModelPrior;

switch type_colorspace,
    case Colorspace.hsv,
        a = coder.load('learned-hiperpriors-on-MODD/EM_parameters_hsv.mat');
    case Colorspace.lab,
        a = coder.load('learned-hiperpriors-on-MODD/EM_parameters_lab.mat');
    case Colorspace.rgb,
        a = coder.load('learned-hiperpriors-on-MODD/EM_parameters_rgb.mat');
    case Colorspace.ycrcb,
        a = coder.load('learned-hiperpriors-on-MODD/EM_parameters_ycrcb.mat');
    case Colorspace.ycrs,
        a = coder.load('learned-hiperpriors-on-MODD/EM_parameters_ycrs.mat');
    otherwise
        error('Invalid colorspace.');
end

parameters_EM = a.parameters_EM;
params = parameters_EM.params;

em_image_size = parameters_EM.em_image_size;

model_prior.mixture = opt_param_space_to_mixture_prior(params', em_image_size);
model_prior.type_colorspace = type_colorspace;
model_prior.is_initialized = true;

end


function mixture = opt_param_space_to_mixture_prior( params, im_size )
 %#codegen

W = im_size(2) ;
H = im_size(1) ;

% project to pixels
n_mu_batch = 4 ;
n_s_batch = 5 ;

mixture = MixtureComponent(3);

Mu_params = params(1:end-3*n_s_batch ) ;
S_params = params(end-3*n_s_batch+1:end) ;

for i = 1 : 3
     mu = [W/2; Mu_params( n_mu_batch*(i-1) + 1 : n_mu_batch*i )] ;  
     mu(2) = mu(2)*H ;
     mixture(i).Mu =mu ;

     S = S_params( n_s_batch*(i-1) + 1 : n_s_batch*i ) ;
     S(1:2) = S(1:2).*[H;W] ;
     mixture(i).Cov = diag(S.^2) ;     
     
     mixture(i).Prec =  inv(mixture(i).Cov) ;
end
end