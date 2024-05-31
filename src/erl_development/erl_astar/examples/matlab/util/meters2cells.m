function [ datac ] = meters2cells( datam,dim_min,res)
%[ datac ] = meters2cells( datam,dim_min,res)
%   datam = [num_dim x num_pts] in meters
%   dim_min = [xmin;ymin;zmin] // the minimum value for each dimensions
%   res = map resolution
%   datac = [num_dim x num_pts] in cells
%
% NOTE: It is the responsibility of the user to ensure that:
%       dim_min <= datam < dim_max
%

if length(res) > 1
    datac = bsxfun(@rdivide,bsxfun(@minus,datam,dim_min),res);
else
    datac = bsxfun(@minus,datam,dim_min)/res;
end

datac = floor(datac)+1;

%datac(datac==0) = 1;
%datac = ceil(datac);

end





%{
if length(res) > 1
    datac = round_down(bsxfun(@rdivide,bsxfun(@minus,datam,dim_min),res)+0.5);
else
    datac = round_down(bsxfun(@minus,datam,dim_min)/res+0.5);
end

datac(datac==0) = 1;

end

% xc = round((xm - xmin) ./ res)+1;
% yc = round((ym - ymin) ./ res)+1;
% zc = round((zm - zmin) ./ res)+1;

function x = round_down(x)
x = ceil(x - 0.5);
end
%}
