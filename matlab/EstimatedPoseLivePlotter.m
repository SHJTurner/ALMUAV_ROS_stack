%EstimatedPoseLivePlotter.m
%Author: Stig Turner
%Mail: sttur14@student.sdu.dk

%call "rosinit" to start roscore and call "rosshutdown" to shutdown the
%roscore
clc; clear all; close all;
%create subscriber for the estimated pose
estimatedPose = rossubscriber('/mavros/vision_pose/pose_cov'); 
number_of_samples = 100

for n = 1:number_of_samples
    

poseData = receive(estimatedPose);
XDataSet(n) = poseData.Pose.Pose.Position.X;
YDataSet(n) = poseData.Pose.Pose.Position.Y;
ZDataSet(n) = poseData.Pose.Pose.Position.Z;
    if n > 1
        delete(px)
        delete(py)
        delete(pz)
    end
px = plot(XDataSet,'r');
py = plot(YDataSet,'g');
pz = plot(ZDataSet,'b');
%if n == 2
%legend('X','Y','Z');
%end
%ylim([0,max(ZDataSet) + 0.2]);
grid on;
hold on;
title('Position');
xlabel('Sample nr.');
ylabel('Meter');
end

while(true)
for n = 2:number_of_samples
    XDataSet(n-1) = XDataSet(n);
    YDataSet(n-1) = YDataSet(n);
    ZDataSet(n-1) = ZDataSet(n);
end

poseData = receive(estimatedPose);
XDataSet(n) = poseData.Pose.Pose.Position.X;
YDataSet(n) = poseData.Pose.Pose.Position.Y;
ZDataSet(n) = poseData.Pose.Pose.Position.Z;
delete(px)
delete(py)
delete(pz)
px = plot(XDataSet,'r');
py = plot(YDataSet,'g');
pz = plot(ZDataSet,'b');
%legend('X','Y','Z');

%ylim([0,max(ZDataSet) + 0.2]);
grid on;
hold on;
title('Position');
xlabel('Sample nr.');
ylabel('Meter');
end