% MAP
res = 0.05;
dev = 24;
%MAP = init_blank_ndmap([-dev;-dev],[dev;dev],res,'logical');
MAP = pgm2map('lt_small_gimped.pgm',res);
MAP.res = res;
% inflate the obstacles
rad = 4*MAP.res;
se = strel('disk',round(rad/MAP.res));
fprnt = getnhood(se);    % logical footprint
rad_cell = floor(size(fprnt,1)/2);
[X,Y] = meshgrid(-rad_cell:rad_cell,-rad_cell:rad_cell);
fprnt = Y(fprnt) + X(fprnt)*MAP.size(1);
MAP.map = map2costmap(MAP.map,100,fprnt);
MAP.obst_thresh = 80;

% sx = [-14.83, -8.128];
% ex = [-20, -2.2];
% startCells = meters2cells(sx.',MAP.min,MAP.res);
% goalCells = meters2cells(ex.',MAP.min,MAP.res);

startCells = meters2cells(MAP.min,MAP.min,MAP.res);
goalCells = meters2cells(MAP.max,MAP.min,MAP.res);
PATH = astarL2Wrapper(double(MAP.map).',startCells.',goalCells.');
PATH = cells2meters(PATH(:,[1,2]),MAP.min.',MAP.res);

figure;
imagesc([MAP.pos{1}(1);MAP.pos{1}(end)],[MAP.pos{2}(1);MAP.pos{2}(end)],MAP.map.');
axis xy;
colormap(1-gray);
hold on;
plot(PATH(:,1),PATH(:,2),'r-');

