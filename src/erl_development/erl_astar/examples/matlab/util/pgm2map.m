function MAP = pgm2map(filename,res,yaml_dx,yaml_dy)
%MAP = pgm2map(filename, res) 
% assumes the image is uint8
%

MAP.map = imread(filename,'pgm');
if size(MAP.map,3) > 1
    MAP.map = MAP.map(:,:,1);
end

MAP.map = MAP.map(end:-1:1,:);  % invert to correct discrepancy between ros and matlab
MAP.map = MAP.map.';
MAP.map = int16(MAP.map);
MAP.map = min(32767-32767*(MAP.map/255),600);

if(numel(res) == 1)
    res = [res;res];
else
    res = res([2 1]);
end

MAP.res = res;


[sy,sx] = size(MAP.map);
MAP.size = [sy;sx];

if (mod(sx,2)==0)
    xmax = (sx+1)/2*res(1);
    xmin = -(sx-1)/2*res(1);
else
    xmax = sx/2*res(1);
    xmin = -sx/2*res(1);
end

if (mod(sy,2)==0)
    ymax = (sy+1)/2*res(2);
    ymin = -(sy-1)/2*res(2);
else
    ymax = sy/2*res(2);
    ymin = -sy/2*res(2);
end
MAP.max = [ymax;xmax];
MAP.min = [ymin;xmin];



% translate
if nargin > 2
    num_pix_x = (yaml_dx - MAP.min(1)) / MAP.res(1);
    if( num_pix_x < 0)
        MAP.map = [MAP.map;zeros(2*abs(num_pix_x),sx);];
    elseif( num_pix_x > 0)
        MAP.map = [zeros(round(2*num_pix_x),sx);MAP.map];
    end
end
if nargin > 3
    num_pix_y = (yaml_dy - MAP.min(2)) / MAP.res(2);
    
    if( num_pix_y < 0)
        MAP.map = [MAP.map, zeros(size(MAP.map,1), 2*abs(num_pix_y))];
    elseif( num_pix_y > 0)
        MAP.map = [zeros(size(MAP.map,1), round(2*num_pix_y)), MAP.map];
    end
end

%MAP.map = [zeros(2*88,736);MAP.map];
%MAP.map = [zeros(size(MAP.map,1), 2*48) MAP.map];


[sy,sx] = size(MAP.map);
MAP.size = [sy;sx];

if (mod(sx,2)==0)
    xmax = (sx+1)/2*res(1);
    xmin = -(sx-1)/2*res(1);
else
    xmax = sx/2*res(1);
    xmin = -sx/2*res(1);
end

if (mod(sy,2)==0)
    ymax = (sy+1)/2*res(2);
    ymin = -(sy-1)/2*res(2);
else
    ymax = sy/2*res(2);
    ymin = -sy/2*res(2);
end
MAP.max = [ymax;xmax];
MAP.min = [ymin;xmin];



MAP.pos = {MAP.min(1)+res(1)/2:res(1):MAP.max(1)-res(1)/2; %y-positions of each pixel of the map
           MAP.min(2)+res(2)/2:res(2):MAP.max(2)-res(2)/2};%x-positions of each pixel of the map
MAP.origin = zeros(2,1);   
MAP.origincells = meters2cells(zeros(2,1),MAP.min(:),MAP.res(:));
%{
MAP.cost_map = double(MAP.map)+1;
MAP.cost_map(MAP.cost_map > int16(inf)) = inf;

MAP.wmap = sparse(MAP.sizey,MAP.sizex);
MAP.wmap_cnt = sparse(MAP.sizey,MAP.sizex);
%}
end
