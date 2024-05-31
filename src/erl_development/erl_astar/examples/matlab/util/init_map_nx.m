function MAP = init_map_nx(devmin,devmax,res,class)
%MAP = init_map_nx(devmin,devmax,res,class)
% devmin = Nx1 = min in each dimension
% devmax = Nx1 = max in each dimension
% res = Nx1 = resolution in each dimension
% class = {sparse, logical, uint8, int16, etc..}

if nargin < 4
   class = 'int16'; 
end

if(any(devmin > devmax))
    error('[init_map_nx]: assert(devmax >= devmin)');
end

MAP.devmin = devmin;
MAP.devmax = devmax;
MAP.res = res;

MAP.origin = (devmax+devmin)/2;              % in meters
MAP.size = oddCeil((devmax-devmin)./res); % always odd
dev = MAP.size.*res/2;
MAP.min = MAP.origin - dev;
MAP.max = MAP.origin + dev;


% num_half_pix_min = oddCeil(2*(origin-devmin)./res);
% num_half_pix_max = oddCeil(2*(devmaxor-origin)./res);
% 
% MAP.min = sign(devmin).*num_half_pix_min.*res/2;
% MAP.max = sign(devmax).*num_half_pix_max.*res/2;
% 
% MAP.size = (num_half_pix_min + num_half_pix_max)/2;

numdim = length(MAP.size);

MAP.pos = cell(numdim,1);
MAP.origincells = zeros(numdim,1);
for k = 1:numdim
    tmp = (MAP.size(k)-1)/2;
    if length(res) > 1
        MAP.pos{k} = MAP.origin(k)+(-tmp:tmp)*res(k);
    else
        MAP.pos{k} = MAP.origin(k)+(-tmp:tmp)*res;
    end
    MAP.origincells(k) = tmp+1;
end


if strcmpi(class,'sparse')
    MAP.map = sparse(prod(MAP.size),1);
else
    if length(MAP.size) == 1
        if strcmpi(class,'logical')
            MAP.map = false(MAP.size,1);
        else
            MAP.map = zeros(MAP.size,1,class);
        end
    else
        if strcmpi(class,'logical')
            MAP.map = false(transpose(MAP.size(:)));
        else
            MAP.map = zeros(transpose(MAP.size(:)),class);
        end
    end
end
%MAP.cost_map = double(MAP.map)+1;
%MAP.origincells = meters2cells(zeros(numdim,1),MAP.min(:),MAP.res);

% MAP.xpos = MAP.xmin+res/2:res:MAP.xmax-res/2; %x-positions of each pixel of the map
% MAP.ypos = MAP.ymin+res/2:res:MAP.ymax-res/2; %y-positions of each pixel of the map
end

function ocx = oddCeil(x)
% returns the first odd integer bigger than x

rndidx = (abs(floor(x) - x) <= eps);

x(rndidx) = floor(x(rndidx));

ocx = ceil(x);

evenidx = (mod(ocx,2) == 0);

ocx(evenidx) = ocx(evenidx) + 1;

end
