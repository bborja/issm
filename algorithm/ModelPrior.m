function model_prior = ModelPrior ()
%#codegen

model_prior = struct(...
    'type_colorspace', Colorspace.hsv,...
    'mixture', MixtureComponent(3));

coder.cstructname(model_prior, 'ModelPrior');

end
