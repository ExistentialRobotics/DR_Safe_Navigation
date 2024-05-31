addpath("../../lib")
addpath("util")
addpath("mex_astar")

% Generate map in 2D
map = zeros(30,30,'int8');
map(15,15) = 1;
% Compute inflated map
infrad = 1;
tic;imap = inflate_map_mex(map,infrad);toc;
% Plan path in 2D
start = [10;10];
goal = [20;20];
eps = 1;
[path2, path_cost2] = jps_2D_mex(start,goal,imap,eps);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%555
% Generate map in 3D
map = zeros(30,30,30,'int8');
map(15,15,15) = 1;
infrad = 1;
tic;imap = inflate_map_mex(map,infrad);toc;

% Plan path in 3D
start = [10;10;10];
goal = [20;20;20];
eps = 1;
[path3, path_cost3] = jps_3D_mex(start,goal,imap,eps);
