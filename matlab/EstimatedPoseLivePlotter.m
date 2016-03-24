%EstimatedPoseLivePlotter.m
%Author: Stig Turner
%Mail: sttur14@student.sdu.dk

%call "rosinit" to start roscore and call "rosshutdown" to shutdown the
%roscore
clc; clear all;
%create subscriber for the estimated pose
estimatedPose = rossubscriber('/monocular_pose_estimator/estimated_pose'); 
number_of_samples = 100

%Recive data
for n = 1:number_of_samples
poseData = receive(estimatedPose,5);
XDataSet(n) = poseData.Pose.Pose.Position.X;
YDataSet(n) = poseData.Pose.Pose.Position.Y;
ZDataSet(n) = poseData.Pose.Pose.Position.Z;
px = plot(XDataSet,'r');
py = plot(YDataSet,'g');
pz = plot(ZDataSet,'b');

%ylim([0,max(ZDataSet) + 0.2]);
grid on;
hold on;
title('Position');
xlabel('Sample nr.');
ylabel('Meter');
end