function costmap = map2costmap(map,obst_thresh,robotfootprint)
% costmap = map2costmap(map,obst_thresh,robotfootprint)
%
% assumes the map is int16
%
% EXAMPLE:
%   rad = 4*MAP.res;
%   se = strel('disk',round(rad/MAP.res));
%   fprnt = getnhood(se);    % logical footprint
%   rad_cell = floor(size(fprnt,1)/2);
%   [X,Y] = meshgrid(-rad_cell:rad_cell,-rad_cell:rad_cell);
%   fprnt = Y(fprnt) + X(fprnt)*MAP.size(1);
%
%  [xx,yy] = ndgrid(-rad_cell:rad_cell,-rad_cell:rad_cell);
%  mask = (xx.^2 + yy.^2)<=rad_cell^2;
%  fprnt = xx(mask) + yy(mask)*size(map,1);
%

costmap = double(map)+1;    % lowerbounds the cost at 1

obst_linidx = find(map > obst_thresh);

if nargin <3
    costmap(obst_linidx) = inf;
else
    config_linidx = bsxfun(@plus,obst_linidx,transpose(robotfootprint));
    
    config_linidx = reshape(config_linidx,[],1);
    
    invalid = (config_linidx > numel(costmap))|(config_linidx < 1);
    
    config_linidx(invalid) = [];
    
    costmap(config_linidx) = inf;
end

end