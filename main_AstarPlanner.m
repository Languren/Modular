clear all
close all
clc

%% Mapa
load occMap

figure
show(occMap)

inflate(occMap,0.2)

figure
show(occMap)

%% un click para inicio (x(1),y(1)), otro click para meta (x(2),y(2))
[x,y] = ginput(2);
x = round(x); y = round(y);

%% A*
planner = plannerAStarGrid(occMap);

% 'Chebyshev', 'Euclidean', 'EulideanSquared', or 'Manhattan'.
planner.GCost = 'Euclidean';
planner.HCost = 'Manhattan';

start = world2grid(occMap,[x(1) y(1)]);
goal = world2grid(occMap,[x(2) y(2)]);

path = plan(planner,start,goal);

%% Resultados
figure
show(planner)

disp('Grid Path:')
path'

disp('World Path:')
xy_path = grid2world(occMap,path)'

save('A_path','xy_path')
