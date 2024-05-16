clear all
clc
close all

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
slamAlg.LoopClosureSearchRadius = 8; % default: 8, [3 15]

slamAlg.MovementThreshold = [0.1 0.1];

%%
tic

while toc<=60
    scanMsg = receive(lidarSub,10);
    Angles = (scanMsg.angle_min:scanMsg.angle_increment:scanMsg.angle_max)';
    Ranges = scanMsg.ranges;

    I = Ranges<0.8;
    scanMsg.ranges(I) = 20;

    scans = rosReadLidarScan(scanMsg);
    addScan(slamAlg,scans);

    [~,poses] = scansAndPoses(slamAlg);

    figure(1)
    cla
    hold on
    grid on
    show(slamAlg);
    title(['Simulation time = ' num2str(toc)])
    drawnow

    [X,Y] = pol2cart(Angles,Ranges);
end

figure
show(slamAlg);
title({'Map of the Environment','Pose Graph'})

[scansSLAM,poses] = scansAndPoses(slamAlg);
occMap = buildMap(scansSLAM,poses,mapResolution,maxLidarRange);

figure
show(occMap);
title('Occupancy Map of Garage')

save('occMap.mat','occMap')

