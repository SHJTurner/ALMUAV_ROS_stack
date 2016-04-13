%EstimatedPosePlotter.m
%Author: Stig Turner
%Mail: sttur14@student.sdu.dk

%% Init
% %connect to roscore or start a new one
 clc; clear; close all;
% rosshutdown; %if connected to ros core then disconnect
% rosinit; %connect to roscore (fails if allready connected)
% %create subscriber for the estimated pose
% estimatedPose = rossubscriber('/monocular_pose_estimator/estimated_pose'); 
% number_of_samples = 100;

bag = rosbag('/home/turner/Workspaces/catkin_ws_ALMUAV/Recordings/120cm_breadboard.bag');
bagselect_Pose = select(bag,'Topic','/monocular_pose_estimator/estimated_pose');
poseData = readMessages(bagselect_Pose);
%Recive data

%poseData{2}
%for n = 1:number_of_samples
%poseData = receive(estimatedPose,5);
TimeStart = 2.5;
TimeEnd = 6.5;
DeltaTime = TimeEnd -TimeStart;
TimeOffsetSec = double(poseData{1}.Header.Stamp.Sec) + (double(poseData{1}.Header.Stamp.Nsec) * 0.000000001);
for n = 1:length(poseData)

        PoseTimeSec(n) = poseData{n}.Header.Stamp.Sec;
        PoseTimeNsec(n) = poseData{n}.Header.Stamp.Nsec;
        NanoSec = double(poseData{n}.Header.Stamp.Nsec) * 0.000000001;
        PoseTime(n) = double(poseData{n}.Header.Stamp.Sec) + NanoSec - TimeOffsetSec - double(TimeStart)-1.0;
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
TimeLabel = 'Seconds';
DistanceLabel = 'Meter';
AngleLabel = 'Radians';
ax = subplot(3,2,1);
plot(PoseTime,PosData(:,1),'r');
grid(ax,'on')
xlabel(TimeLabel);
ylabel(DistanceLabel);
xlim([0,DeltaTime]);
mu = mean(PosData(:,1));
ylim([mu-0.03,mu+0.03]);
title('X')
hline = refline([0 mu]);
hline.Color = [0.5 0.5 0.5];
set(hline,'LineStyle','--');
hline.LineWidth = 1.5;
uistack(hline,'bottom')


ax = subplot(3,2,3);
plot(PoseTime,PosData(:,2),'g')
grid(ax,'on')
xlabel(TimeLabel)
ylabel(DistanceLabel)
xlim([0,DeltaTime]);
mu = mean(PosData(:,2));
ylim([mu-0.03,mu+0.03]);
title('Y')
hline = refline([0 mu]);
hline.Color = [0.5 0.5 0.5];
set(hline,'LineStyle','--');
hline.LineWidth = 1.5;
uistack(hline,'bottom')

ax = subplot(3,2,5);
plot(PoseTime,PosData(:,3),'b')
grid(ax,'on')
xlabel(TimeLabel)
ylabel(DistanceLabel)
xlim([0,DeltaTime]);
mu = mean(PosData(:,3));
ylim([mu-0.03,mu+0.03]);
title('Z')
hline = refline([0 mu]);
hline.Color = [0.5 0.5 0.5];
set(hline,'LineStyle','--');
hline.LineWidth = 1.5;
uistack(hline,'bottom')

ax = subplot(3,2,2);
plot(PoseTime,EuOriData(:,1),'r')
grid(ax,'on')
xlabel(TimeLabel)
ylabel(AngleLabel)
xlim([0,DeltaTime]);
ylim([-3.14,3.14]);
title('Pitch')
mu = mean(EuOriData(:,1));
hline = refline([0 mu]);
hline.Color = [0.5 0.5 0.5];
set(hline,'LineStyle','--');
hline.LineWidth = 1.5;
uistack(hline,'bottom')

ax = subplot(3,2,4);
plot(PoseTime,EuOriData(:,2),'g')
grid(ax,'on')
xlabel(TimeLabel)
ylabel(AngleLabel)
xlim([0,DeltaTime]);
ylim([-3.14,3.14]);
title('Roll')
mu = mean(EuOriData(:,2));
hline = refline([0 mu]);
hline.Color = [0.5 0.5 0.5];
set(hline,'LineStyle','--');
hline.LineWidth = 1.5;
uistack(hline,'bottom')

ax = subplot(3,2,6);
plot(PoseTime,EuOriData(:,3),'b')
grid(ax,'on')
xlabel(TimeLabel)
ylabel(AngleLabel)
xlim([0,DeltaTime]);
ylim([-3.14,3.14]);
title('Yaw')
mu = mean(EuOriData(:,3));
hline = refline([0 mu]);
hline.Color = [0.5 0.5 0.5];
set(hline,'LineStyle','--');
hline.LineWidth = 1.5;
uistack(hline,'bottom')


%% Plot 3D of path
figure;
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
figure
[m,n] = size(PosData);
scatter3(PosData(:,1),PosData(:,2),PosData(:,3),'x','r');
hold on
for(c = 1:m)
R = eul2rotm(EuOriData(c,:));
vec = [0 0 0.01] * R;
quiver3(PosData(c,1),PosData(c,2),PosData(c,3),vec(1),vec(2),vec(3),'LineWidth',1,'MaxHeadSize',1);
hold on
end
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Orientatin plot of detections');
xlim([min(PosData(:,1))-0.015,max(PosData(:,1))+0.015]);
ylim([min(PosData(:,2))-0.015,max(PosData(:,2))+0.015]);
zlim([min(PosData(:,3))-0.015,max(PosData(:,3))+0.015]);


