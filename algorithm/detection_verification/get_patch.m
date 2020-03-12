function im = get_patch(img, c, scale, template_size)
% extract constant-size patch from the image (if region goes out of the 
% image use border pixels on the pixels outside image)
% INPUT:
% img: image
% c: patch center position [x, y]
% scale: scale of the patch (scalar, use 1 if not estimating the scale)
% template_size: size of the patch [width, height]

% calculate size of the patch
w = floor(scale*template_size(1));
h = floor(scale*template_size(2));

% calcualte indexes - positions of the extraction
xs = floor(c(1)) + (1:w) - floor(w/2);
ys = floor(c(2)) + (1:h) - floor(h/2);

% check if something out of image
xs(xs < 1) = 1;
ys(ys < 1) = 1;
xs(xs > size(img,2)) = size(img,2);
ys(ys > size(img,1)) = size(img,1);

% extract from image
im = img(ys, xs, :);

end  % endfunction