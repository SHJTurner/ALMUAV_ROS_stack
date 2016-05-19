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
path = '/home/turner/Workspaces/catkin_ALMUAV_workspace/_2016-05-17-11-58-33';
bag = rosbag(strcat(path,'.bag'));%'/home/turner/Workspaces/catkin_ws_ALMUAV/Recordings/120cm_breadboard.bag');
bagselect_Pose = select(bag,'Topic','/mavros/vision_pose/pose_cov');%'/monocular_pose_estimator/estimated_pose');
poseData = readMessages(bagselect_Pose);
%Recive data

%poseData{2}
%for n = 1:number_of_samples
%poseData = receive(estimatedPose,5);
TimeStart = 0.0;
TimeEnd = 5.0;
DeltaTime = TimeEnd -TimeStart;
TimeOffsetSec = double(poseData{1}.Header.Stamp.Sec) + (double(poseData{1}.Header.Stamp.Nsec) * 0.000000001);
for n = 1:length(poseData)

        PoseTimeSec(n) = poseData{n}.Header.Stamp.Sec;
        PoseTimeNsec(n) = poseData{n}.Header.Stamp.Nsec;
        NanoSec = double(poseData{n}.Header.Stamp.Nsec) * 0.000000001;
        PoseTime(n) = double(poseData{n}.Header.Stamp.Sec) + NanoSec - TimeOffsetSec - double(TimeStart);
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
for n=1:length(EuOriData)
    EuOriData(n,1) = EuOriData(n,1)*(180/pi);
    EuOriData(n,2) = EuOriData(n,2)*(180/pi);
    EuOriData(n,3) = EuOriData(n,3)*(180/pi);
end


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
%limitY =[-3.14,3.14];
limitY = [-90,90];
limitYmeter = 0.05;
limitYmeterZ = 0.15

figure
TimeLabel = 'time [s]';
DistanceLabel = 'position [m]';
AngleLabel = 'angle [deg]';
ax = subplot(3,2,1);
plot(PoseTime,PosData(:,1),'r');
grid(ax,'on')
xlabel(TimeLabel);
ylabel(DistanceLabel);
xlim([0,DeltaTime]);
mu = mean(PosData(:,1));
ylim([mu-limitYmeter,mu+limitYmeter]);
title('X')
hline = refline([0 mu]);
hline.Color = [0.5 0.5 0.5];
set(hline,'LineStyle','--');
hline.LineWidth = 1.5;
uistack(hline,'bottom')
STDx = std(PosData(:,1))
MEANx = mean(PosData(:,1))

ax = subplot(3,2,3);
plot(PoseTime,PosData(:,2),'g')
grid(ax,'on')
xlabel(TimeLabel)
ylabel(DistanceLabel)
xlim([0,DeltaTime]);
mu = mean(PosData(:,2));
ylim([mu-limitYmeter,mu+limitYmeter]);
title('Y')
hline = refline([0 mu]);
hline.Color = [0.5 0.5 0.5];
set(hline,'LineStyle','--');
hline.LineWidth = 1.5;
uistack(hline,'bottom')
STDy = std(PosData(:,2))
MEANy = mean(PosData(:,2))

ax = subplot(3,2,5);
plot(PoseTime,PosData(:,3),'b')
grid(ax,'on')
xlabel(TimeLabel)
ylabel(DistanceLabel)
xlim([0,DeltaTime]);
mu = mean(PosData(:,3));
ylim([mu-limitYmeterZ,mu+limitYmeterZ]);
title('Z')
hline = refline([0 mu]);
hline.Color = [0.5 0.5 0.5];
set(hline,'LineStyle','--');
hline.LineWidth = 1.5;
uistack(hline,'bottom')
STDz = std(PosData(:,3))
MEANz = mean(PosData(:,3))


ax = subplot(3,2,2);
plot(PoseTime,EuOriData(:,1),'r')
grid(ax,'on')
xlabel(TimeLabel)
ylabel(AngleLabel)
xlim([0,DeltaTime]);
ylim(limitY);
title('Roll')
mu = mean(EuOriData(:,1));
hline = refline([0 mu]);
hline.Color = [0.5 0.5 0.5];
set(hline,'LineStyle','--');
hline.LineWidth = 1.5;
uistack(hline,'bottom')
STDr = std(EuOriData(:,1))
MEANr = mean(EuOriData(:,1))

ax = subplot(3,2,4);
plot(PoseTime,EuOriData(:,2),'g')
grid(ax,'on')
xlabel(TimeLabel)
ylabel(AngleLabel)
xlim([0,DeltaTime]);
ylim(limitY);
title('Pitch')
mu = mean(EuOriData(:,2));
hline = refline([0 mu]);
hline.Color = [0.5 0.5 0.5];
set(hline,'LineStyle','--');
hline.LineWidth = 1.5;
uistack(hline,'bottom')
STDp = std(EuOriData(:,2))
MEANp = mean(EuOriData(:,2))

ax = subplot(3,2,6);
plot(PoseTime,EuOriData(:,3),'b')
grid(ax,'on')
xlabel(TimeLabel)
ylabel(AngleLabel)
xlim([0,DeltaTime]);0
ylim([-180,180]);
title('Yaw')
mu = mean(EuOriData(:,3));
hline = refline([0 mu]);
hline.Color = [0.5 0.5 0.5];
set(hline,'LineStyle','--');
hline.LineWidth = 1.5;
uistack(hline,'bottom')
STDyaw = std(EuOriData(:,3))
MEANyaw = mean(EuOriData(:,3))

%% export
snam = 'XYZRPY';
s=hgexport('readstyle',snam);
hgexport(gcf,strcat(path,'.eps'));

fid=fopen(strcat(path,'.txt'),'w');
data = [STDx, MEANx, STDy, MEANy, STDz, MEANz, STDr, MEANr, STDp, MEANp, STDyaw, MEANyaw];
formatSpec = 'X: standard deviation: %2.4f, Mean:  %2.4f\nY: standard deviation: %2.4f, Mean:  %2.4f\nZ: standard deviation: %2.4f, Mean:  %2.4f\nRoll: standard deviation: %2.4f, Mean:  %2.4f\nPitch: standard deviation: %2.4f, Mean:  %2.4f\nYaw: standard deviation: %2.4f, Mean:  %2.4f\n';
fprintf( fid,formatSpec,data);
fclose(fid);
 close all;
% %% Plot 3D of path
% figure;
% scatter3(PosData(:,1),PosData(:,2),PosData(:,3),'x','r');
% hold on
% plot3(PosData(:,1),PosData(:,2),PosData(:,3),'g');
% hold on
% xlabel('Meter');
% ylabel('Meter');
% zlabel('Meter');
% title('Plot of detections and path');
% xlim([-0.75,0.75]);
% ylim([-0.75,0.75]);
% zlim([-0.15,1.5]);
% R = [1 0 0; 0 1 0; 0 0 1];
% plotCamera('Location',[0 0 -0.10],'Orientation',R,'Size',0.05,'Color',[0,0,0]);
% 
% %% Close up orientation plot
% %figure
% %[m,n] = size(PosData);
% %scatter3(PosData(:,1),PosData(:,2),PosData(:,3),'x','r');
% %hold on
% %for(c = 1:m)
% %R = eul2rotm(EuOriData(c,:));
% %vec = [0 0 0.01] * R;
% %quiver3(PosData(c,1),PosData(c,2),PosData(c,3),vec(1),vec(2),vec(3),'LineWidth',1,'MaxHeadSize',1);
% %hold on
% %end
% %xlabel('X');
% %ylabel('Y');
% %zlabel('Z');
% %title('Orientatin plot of detections');
% %xlim([min(PosData(:,1))-0.015,max(PosData(:,1))+0.015]);
% %ylim([min(PosData(:,2))-0.015,max(PosData(:,2))+0.015]);
% %zlim([min(PosData(:,3))-0.015,max(PosData(:,3))+0.015]);
% 
% 
