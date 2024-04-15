clear all
clc
close all

rosshutdown
% rosinit("http://192.168.100.95:11311")
% lidarSub = rossubscriber("/scan","sensor_msgs/LaserScan");

detectNode = ros2node("/detection");
pause(5)
lidarSub = ros2subscriber(detectNode,"/scan","sensor_msgs/LaserScan","Reliability","besteffort","Durability","volatile","Depth",5);
%lidarSub = ros2subscriber(detectNode,"/scan");
pause(5)
% SLAM
maxLidarRange = 12;  % en metros
mapResolution = 20; % Para mapa en rejillas de ocupacion (pixeles/metro) 
slamAlg = lidarSLAM(mapResolution,maxLidarRange);

% The following loop closure parameters are set empirically: 
% Higher loop closure threshold helps reject false positives
slamAlg.LoopClosureThreshold = 100; % default: 100, [50 500]
% Tune this distance based on your environment and the expected vehicle trajectory.
slamAlg.LoopClosureSearchRadius = 8; % default: 8, [3 15]

slamAlg.MovementThreshold = [0.1 0.1];

%%
tic

while toc<=60
   % [scanData,status,statustext] = receive(lidarSub,10);
    scanMsg = receive(lidarSub,10);
   % Angles = (scanData.angle_min:scanData.angle_increment:scanData.angle_max)';
   % Ranges = scanData.ranges;

    Angles = (scanMsg.angle_min:scanMsg.angle_increment:scanMsg.angle_max)';
    Ranges = scanMsg.ranges;
    scans = rosReadLidarScan(scanMsg);
  %  scans = lidarScan(Ranges,Angles);
	addScan(slamAlg,scans);



% [~,poses] = scansAndPoses(slamAlg % p = poses(end,:)';


figure(1)
    cla
    hold on
    grid on
    show(slamAlg);
    title(['Simulation time = ' num2str(toc)])
    drawnow
    
    [X,Y] = pol2cart(Angles,Ranges);
    
    % figure(2)
    % cla
    % hold on
    % grid on
    % scatter(X,Y,'filled')
    % xlabel('x')
    % ylabel('y')
    % axis([-3 3 -3 3])
    % drawnow
end

figure
show(slamAlg);
title({'Map of the Environment','Pose Graph'})

[scansSLAM,poses] = scansAndPoses(slamAlg);
occMap = buildMap(scansSLAM,poses,mapResolution,maxLidarRange);

figure
show(occMap);
title('Occupancy Map of Garage')

