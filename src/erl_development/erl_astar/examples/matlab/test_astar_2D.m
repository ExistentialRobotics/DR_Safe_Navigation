
% % Simple test
% res = [1;1];
% MAP = init_map_nx([0;0],[5;3],res);
% MAP.map(1,2) = 1;
% 
% start = [0.5;0.5];
% goal = [0.5;2.5];
% 
% [PATH_m, cost, traj_id,len_id] = astar_mex(start,goal,MAP.min,MAP.max,MAP.res,MAP.map);

addpath("../../lib")
addpath("util")
addpath("mex_astar")

% 0.76
res = 0.05;
tmp = pgm2map('lt_small_gimped.pgm',res);
MAP = init_map_nx(tmp.min,tmp.max,tmp.res);
MAP.map(1:end-1,:) = tmp.map; clear('tmp');
%MAP.map = double(MAP.map) + 1;
MAP.obst_thresh = 80;


start = [-20.95;20.8];
goal = [6.4;-14.3];

fprintf('Starting...\n');

%profile on
tic;
[PATH, cost] = test_astar_2D_mex(start,goal,MAP.min,MAP.max,MAP.res,MAP.map);
toc;
%profile viewer

fprintf('my A* cost = %f\n',cost);

tic;
startCells = meters2cells(start,MAP.min,MAP.res);
goalCells = meters2cells(goal,MAP.min,MAP.res);
[PATH1_c,cost_mat] = astarL2Wrapper((double(MAP.map) + 1).',startCells.',goalCells.');
cost1 = cost_mat(PATH1_c(end,2),PATH1_c(end,1))-1; % subtract one because map cost was inflated to be nonzero
PATH1 = cells2meters(PATH1_c(:,[1,2]),MAP.min.',MAP.res.');
toc;

fprintf('fast A* cost = %f\n',cost1);



%% Display
figure;
imagesc([MAP.pos{1}(1);MAP.pos{1}(end)],[MAP.pos{2}(1);MAP.pos{2}(end)],MAP.map.');
axis xy;
grayc = gray;
colormap(grayc(end:-1:1,:));

hold on;
plot(PATH1(:,1),PATH1(:,2),'r-');
plot(PATH(1,:),PATH(2,:),'b-');


% % Save map to file
% fileID = fopen('large_map.cfg','w');
% fprintf(fileID,'%d ',MAP.map);
% fclose(fileID);
