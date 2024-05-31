function [path,cost] = astar_L2_Wrapper(costMap,start,goal)
% INPUT:
%   costMap = y-by-x = class double! AND strictly positive (> 0)
%   start = (x,y)
%   goal = (x,y)
%
% OUTPUT
%   path = (x,y,yaw)
%


% Determines the state closest to the goal in L2 norm that has a finite
% gVal and sets it as the new goal!
if(nargout > 1)
    [cost,xPrev,yPrev,goal] = astar_L2_mex(costMap,start,goal);
else
    [~,xPrev,yPrev,goal] = astar_L2_mex(costMap,start,goal);
end

path = goal;

xx = goal(1);
yy = goal(2);

while ~((xx == start(1)) && (yy == start(2)))
    
    xloc = xPrev(yy,xx);
    yloc = yPrev(yy,xx);
    
    path = [xloc,yloc;path];
    
    xx = xloc;
    yy = yloc;
    
end


end

