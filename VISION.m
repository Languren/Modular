clear all
clc
close all
point=[];
coords=[];
xy_path=[];
W=0;
b = bluetooth('ESP32_BT_2', 1); %Nombre del dispositivo Bluetooth y canal

v1 = 0;
v2 = 0;
v3 = 0;
v4 = 0;

writeline(b,[num2str(v1) ',' num2str(v2) ',' num2str(v3) ',' num2str(v4) ',' num2str(0)]);
disp('Mete las coordenadas')
fopen(b);
% Para recibir datos
data = fscanf(b); % fscanf lee los datos como una cadena
x = size(data,2);
for i=1:1:x
    if (data(i)~=' ')
        point(i) = data(i);
    end
end
%%
% point = ['1','2','3']
%4.3 X -1.3Y
%3X 2Y
%7.3X 1.5Y
for a=1:1:size(point,2)
    if point(a) == '1'
        coords(a,:) = [5,-1.3];
    elseif point(a) == '2'
        coords(a,:) = [4.5,2.7];
    elseif point(a) == '3'
        coords(a,:) = [-0.7,2.7];
    elseif point(a) == '4'
        coords(a,:) = [2,0];
    elseif point(a) == '5'
        coords(a,:) = [1.5,0];
    elseif point(a) == '6'
        coords(a,:) = [1,0];
    elseif point(a) == '7'
        coords(a,:) = [7,0];
    elseif point(a) == '8'
        coords(a,:) = [8,0];
    elseif point(a) == '9'
        coords(a,:) = [9,0];
    elseif point(a) == 'A'
        coords(a,:) = [0.5,0];
    elseif point(a) == 'B'
        coords(a,:) = [11,0];
    elseif point(a) == 'C'
        coords(a,:) = [12,0];
    elseif point(a) == 'D'
        coords(a,:) = [13,0];
    end
end

load occMap

% figure
% show(occMap)

inflate(occMap,0.8)

% figure
% show(occMap)

% un click para inicio (x(1),y(1)), otro click para meta (x(2),y(2))
% [x,y] = ginput(2);
% x = round(x); y = round(y);

% A*
planner = plannerAStarGrid(occMap);

% 'Chebyshev', 'Euclidean', 'EulideanSquared', or 'Manhattan'.
planner.GCost = 'Euclidean';
planner.HCost = 'Manhattan';

% start = [0,0];
% middle = coords(1,:);
% goal = coords(2,:);
%
% start = world2grid(occMap,[x(1) y(1)]);
% goal = world2grid(occMap,[x(2) y(2)]);

% spath = Suavisar_Trayectoria(path');
% ipath = Interpolacion_Simple(spath,1);
xy_path = plan(planner,[0,0],coords(1,:),'world');
for i=1:1:(size(coords',2)-1)
    start = coords(i,:);
    goal = coords(i+1,:);
    path = plan(planner,start,goal,'world');
    xy_path =  [xy_path; path];
    figure
    show(planner)

end

path = plan(planner,goal,[0,0],'world');
xy_path =  [xy_path; path];
%spath = Suavisar_Trayectoria(xy_path');

% figure
% grid on
% plot(spath(1,:),spath(2,:),'bo-','LineWidth',2,'MarkerSize',10)
% hold on
% plot(coords(1,1),coords(1,2),'rx','MarkerSize',10)
% plot(coords(2,1),coords(2,2),'rx','MarkerSize',10)
% plot(coords(3,1),coords(3,2),'rx','MarkerSize',10)
%%
clear b
b = bluetooth('ESP32_BT_2', 1);
K=diag([1500 1500 500]); %1200-3000
Kv = diag([200 200 200]);%200-600
u = zeros(1,3);
X = xy_path(:, 1);
Y = xy_path(:, 2);

n = size(X);
N = n(1);
i = 1;
o = 2;
L = 0.13;
l = 0.33;
prev_error = 0;
rosshutdown

detectNode = ros2node("/detection");
pause(5)
lidarSub = ros2subscriber(detectNode,"/scan","sensor_msgs/LaserScan","Reliability","besteffort","Durability","volatile","Depth",5);
pause(5)
% SLAM
maxLidarRange = 12;  % en metros
mapResolution = 20; % Para mapa en rejillas de ocupacion (pixeles/metro)
slamAlg = lidarSLAM(mapResolution,maxLidarRange);

% The following loop closure parameters are set empirically:
% Higher loop closure threshold helps reject false positives
slamAlg.LoopClosureThreshold = 300; % default: 100, [50 500]
% Tune this distance based on your environment and the expected vehicle trajectory.
slamAlg.LoopClosureSearchRadius = 3; % default: 8, [3 15]

slamAlg.MovementThreshold = [0.1 0.1];

% 1 hasta la proxima
% 2 punto llegada
% 3 inicio
tic
writeline(b,[num2str(0) ',' num2str(0) ',' num2str(0) ',' num2str(0) ',' num2str(3)]);
while toc<=9999

    scanMsg = receive(lidarSub,10);

    Angles = (scanMsg.angle_min:scanMsg.angle_increment:scanMsg.angle_max)';
    Ranges = scanMsg.ranges;
    scans = rosReadLidarScan(scanMsg);
    addScan(slamAlg,scans);
    if W==0
        pd = [X(i);Y(i); 0];
    end
    if W==1
        pd = [X(i);Y(i); pi/2];
    end
    if W==2
        pd = [X(i);Y(i); pi];
    end
    if W==3
        pd = [X(i);Y(i); 3*pi/2];
    end
    [~,poses] = scansAndPoses(slamAlg);
    p = poses(end,:)'
    pp=[u(1) u(2) u(3)]';
    
    if(p(3)<(-pi/8))
        p(3) = p(3) + 2*pi;
    end
    t=toc;
    error = pd - p;
    error_derivative = (error - prev_error) / toc;
    prev_error = error;
    u=K*error + Kv*error_derivative; %accion de control
    alpha=p(3)+(pi/4);

    v = [sqrt(2)*sin(alpha) -sqrt(2)*cos(alpha) -(L+l); ...
        sqrt(2)*cos(alpha) sqrt(2)*sin(alpha) (L+l); ...
        sqrt(2)*cos(alpha) sqrt(2)*sin(alpha) -(L+l); ...
        sqrt(2)*sin(alpha) -sqrt(2)*cos(alpha) (L+l)]*u;
    if norm(pd-p)<0.45
        i = i+1;
        o = o+1;
    end
    if i>=N
        if i>N
            writeline(b,[num2str(0) ',' num2str(0) ',' num2str(0) ',' num2str(0) ',' num2str(1)]);
            return
        end
        i=N;
    end
    if o<N
     if (X(i)==X(o)) && (Y(i)==Y(o))
        writeline(b,[num2str(0) ',' num2str(0) ',' num2str(0) ',' num2str(0) ',' num2str(2)]);
        disp('pausa')
        pause(10)
        writeline(b,[num2str(0) ',' num2str(0) ',' num2str(0) ',' num2str(0) ',' num2str(3)]);
        i = i+2;
        o = o+2;
        W = W+1;
     end
    end

    v1 = round(v(1));
    v2 = round(v(2));
    v3 = round(v(3));
    v4 = round(v(4));

    v1 = max(v1,-600); v1 = min(v1, 600);
    v2 = max(v2,-600); v2 = min(v2, 600);
    v3 = max(v3,-600); v3 = min(v3, 600);
    v4 = max(v4,-600); v4 = min(v4, 600);

    writeline(b,[num2str(v1) ',' num2str(v2) ',' num2str(v3) ',' num2str(v4) ',' num2str(0)]);

    fprintf('si %d %d %d %d \n', v1, v2, v3, v4)
end
    plot(p);
    show(slamAlg);
    title({'Map of the Environment','Pose Graph'})
    drawnow
    plot(X,Y);

function [ps] = Suavisar_Trayectoria (p)
    ps = p;
    N = 10;
    M = size(p,2);

    alpha = 0.2;
    beta = 0.6;

    for j=1:N
        for i=2:(M-1)
            ps(:,i) = ps(:,i) + alpha*(p(:,i)-ps(:,i)) + beta*(ps(:,i+1)+ps(:,i-1)-2*ps(:,i));
        end
    end
end