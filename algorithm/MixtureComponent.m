function mixture = MixtureComponent(num)
%#codegen

mixture_component = struct(...
    'Mu', [], ...
    'Cov', [], ...
    'w', 0, ...
    'Prec', []);

mixture = repmat(mixture_component, num, 1);

coder.cstructname(mixture, 'MixtureComponent');
coder.varsize('mixture(:).Mu','mixture(:).Cov','mixture(:).Prec');

end



