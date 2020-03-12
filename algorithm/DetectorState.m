function state = DetectorState ()
%#codegen

state = struct(...
    'use_prior_on_mixture', true, ... % Use prior on mixture
    'use_uniform_component', true, ... % Use uniform component in mixture
    'colorspace', Colorspace.ycrcb, ... % Colorspace
    'em_min_likelihood_delta', 1e-2, ... % Smallest change in likelihood to stop EM
    'em_max_steps', 10, ... % Maximum number of EM iterations % set to 20 in practice
    ...
    'is_initialized', false, ...
    'prior_mixture', MixtureComponent(3), ...
    'current_mixture', MixtureComponent(3), ...
    'em_image_size', [ 0 0 ], ...
    'spatial_data', [], ...
    'PI_i', []);

coder.cstructname(state, 'DetectorState');
coder.varsize('state.spatial_data');
coder.varsize('state.PI_i', [], [ 1 1 1 ]);

end


