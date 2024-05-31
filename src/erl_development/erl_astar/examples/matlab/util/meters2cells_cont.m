function [ datac ] = meters2cells_cont( datam,dim_min,res)
%UNTITLED11 Summary of this function goes here
%   datam = [num_dim x num_pts] in meters
%   dim_min = [xmin;ymin;zmin] // the minimum value for each dimensions
%   res = map resolution
%   datac = [num_dim x num_pts] in cells

if length(res) > 1
    datac = bsxfun(@rdivide,bsxfun(@minus,datam,dim_min),res)+0.5;
else
    datac = bsxfun(@minus,datam,dim_min)/res+0.5;
end

end