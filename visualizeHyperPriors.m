function visualizeHyperPriors(model_prior, prior_number)
    mu_x = model_prior.mixture(prior_number).Mu(1);
    mu_y = model_prior.mixture(prior_number).Mu(2);
    
    cov_x = model_prior.mixture(prior_number).Cov(1,1);
    cov_y = model_prior.mixture(prior_number).Cov(2,2);
    
    hyper_prior = zeros(50, 50, 3);
    
    for i = 1 : 50
        for j = 1 : 50
            hyper_prior(i,j,prior_number) = (1 / (2 * pi * cov_x * cov_y)) * exp(-( (j - mu_x)^2 / (2 * cov_x^2) + (i - mu_y)^2 / (2 * cov_y^2) ));              
        end
    end
    
    %max_val = max(max(hyper_prior(:,:,1)));
    %hyper_prior(:,:,prior_number) = hyper_prior(:,:,prior_number) / max_val * 255;

    figure(5); clf;
    imagesc(hyper_prior(:,:,prior_number)); axis equal; axis tight; colormap gray;

end