
res = [0.1;0.1;2.0];
margin = 0.3;
map = nxOccGrid.loadMap('maps/map0.txt', res, margin);

vmin = -3; vmax = 3; vres = 0.25;
MAP = init_map_nx([map.devmin;vmin;vmin;vmin],[map.devmax;vmax;vmax;vmax],[map.res;vres;vres;vres],'sparse');

% acceleration id 0 means acceleration is [0;0;0]
start = [0.0; -4.9; 0.2; 0; 0; 0; 0];
goal= [6.0; 12.0; 3.0; 0; 0; 0; 0];


fprintf('Starting...\n');

%profile on
tic;
[PATH, cost] = astar_3D_mex(start,goal,MAP.min,MAP.max,MAP.res,uint16(map.map));
toc;
%profile viewer

fprintf('my A* cost = %f\n',cost);

figure;
h.map = map.plot();
hold on;

