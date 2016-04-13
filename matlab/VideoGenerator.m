%EstimatedPosePlotter.m
%Author: Stig Turner
%Mail: sttur14@student.sdu.dk

%% Init
% %connect to roscore or start a new one
 clc; clear; close all;

bag = rosbag('/home/turner/Workspaces/catkin_ws_ALMUAV/Recordings/Indoor_UAVtest_010416_withEstimatedPose.bag');
bagselect_Pose = select(bag,'Topic','/monocular_pose_estimator/estimated_pose');
bagselect_Video = select(bag,'Topic','/monocular_pose_estimator/image_with_detections');
poseData = readMessages(bagselect_Pose);
videoData = readMessages(bagselect_Video);

%Load Video
for n = 1:length(videoData)
    frameTime(n) = videoData{n}.Header.Stamp.Sec;
    frameData{n} = readImage(videoData{n});
end

%Load PosData
for n = 1:length(poseData)
    PoseTime(n) = poseData{n}.Header.Stamp.Sec;
    PosData(n,1) = poseData{n}.Pose.Pose.Position.X;
    PosData(n,2) = poseData{n}.Pose.Pose.Position.Y;
    PosData(n,3) = poseData{n}.Pose.Pose.Position.Z;
    QOriData(n,1) = poseData{n}.Pose.Pose.Orientation.X;
    QOriData(n,2) = poseData{n}.Pose.Pose.Orientation.Y;
    QOriData(n,3) = poseData{n}.Pose.Pose.Orientation.Z;
    QOriData(n,4) = poseData{n}.Pose.Pose.Orientation.W;
end
%Shutdown or disconnect from roscore
%rosshutdown;
%Calc orientation in Eular
EuOriData = quat2eul(QOriData);

%% Plot data
% figure
% px = plot(PosData(:,1),'r');
% hold on;
% py = plot(PosData(:,2),'g');
% hold on;
% pz = plot(PosData(:,3),'b');
% ylim([min(min(PosData))-0.1 , max(max(PosData)) + 0.2]);
% title('Position')
% xlabel('Sample nr.')
% ylabel('Meter')
% hold off

%% Plot X,Y,Z,Pitch,Roll,Yaw
figure
ax = subplot(3,2,1);
plot(PosData(:,1),'r')
grid(ax,'on')
xlabel('Sample nr.');
ylabel('Meter');
%mu = mean(PosData(:,1));
%ylim([mu-0.05,mu+0.05]);
title('X')

ax = subplot(3,2,3);
plot(PosData(:,2),'g')
grid(ax,'on')
xlabel('Sample nr.')
ylabel('Meter')
%mu = mean(PosData(:,2));
%ylim([mu-0.05,mu+0.05]);
title('Y')

ax = subplot(3,2,5);
plot(PosData(:,3),'b')
grid(ax,'on')
xlabel('Sample nr.')
ylabel('Meter')
%mu = mean(PosData(:,3));
%ylim([mu-0.05,mu+0.05]);
title('Z')

ax = subplot(3,2,2);
plot(EuOriData(:,1),'r')
grid(ax,'on')
xlabel('Sample nr.')
ylabel('Radian')
title('Pitch')

ax = subplot(3,2,4);
plot(EuOriData(:,2),'g')
grid(ax,'on')
xlabel('Sample nr.')
ylabel('Radian')
title('Roll')

ax = subplot(3,2,6);
plot(EuOriData(:,3),'b')
grid(ax,'on')
xlabel('Sample nr.')
ylabel('Radian')
title('Yaw')



%% Plot 3D of path
v = VideoWriter('/home/turner/Workspaces/catkin_ws_ALMUAV/Recordings/out.avi');
open(v);
fig = figure('Position', [0, 0, 1280,720]);
PoseTimeIndex = 1;
for n=1:length(frameData)
    
    if PoseTime(PoseTimeIndex) <= frameTime(n) && PoseTimeIndex < length(PoseTime)
        PoseTimeIndex = PoseTimeIndex + 1;        
    end
    
    subplot(1,2,1)
    plot3(PosData(1:PoseTimeIndex,1),PosData(1:PoseTimeIndex,2),PosData(1:PoseTimeIndex,3),'b','LineWidth',2,'Marker','x','MarkerEdgeColor','r','MarkerSize',2);
    grid on
    hold on
    %R = [1 0 0; 0 1 0; 0 0 1];
    %plotCamera('Location',[0 0 -0.10],'Orientation',R,'Size',0.05,'Color',[0,0,0]);
    xlabel('Meter');
    ylabel('Meter');
    zlabel('Meter');
    xlim([-0.75,0.75]);
    ylim([-0.75,0.75]);
    zlim([-0.15,1.5]);
    title('Position');
    
    subplot(1,2,2), subimage(frameData{n});
    drawnow;
    F = getframe(fig);
    writeVideo(v,F);
    
end
close(v)

subplot(1,2,1);
scatter3(PosData(:,1),PosData(:,2),PosData(:,3),'x','r');
hold on
plot3(PosData(:,1),PosData(:,2),PosData(:,3),'g');
hold on
xlabel('Meter');
ylabel('Meter');
zlabel('Meter');
title('Plot of detections and path');
xlim([-0.75,0.75]);
ylim([-0.75,0.75]);
zlim([-0.15,1.5]);
R = [1 0 0; 0 1 0; 0 0 1];
plotCamera('Location',[0 0 -0.10],'Orientation',R,'Size',0.05,'Color',[0,0,0]);



%% Close up orientation plot
% figure
% [m,n] = size(PosData);
% scatter3(PosData(:,1),PosData(:,2),PosData(:,3),'x','r');
% hold on
% for(c = 1:m)
% R = eul2rotm(EuOriData(c,:));
% vec = [0 0 0.01] * R;
% quiver3(PosData(c,1),PosData(c,2),PosData(c,3),vec(1),vec(2),vec(3),'LineWidth',1,'MaxHeadSize',1);
% hold on
% end
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% title('Orientatin plot of detections');
% xlim([min(PosData(:,1))-0.015,max(PosData(:,1))+0.015]);
% ylim([min(PosData(:,2))-0.015,max(PosData(:,2))+0.015]);
% zlim([min(PosData(:,3))-0.015,max(PosData(:,3))+0.015]);


