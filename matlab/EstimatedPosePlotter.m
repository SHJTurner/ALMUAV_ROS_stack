%"rosinit" %Init ROS-core (Do it from matlab conole)
clc; clear all;
estimatedPose = rossubscriber('/monocular_pose_estimator/estimated_pose'); %create subscriber for the estimated pose
number_of_samples = 10

%Recive data
%XDataSet = zeros(1,number_of_samples);
%YDataSet = zeros(1,number_of_samples);
%ZDataSet = zeros(1,number_of_samples);
for n = 1:number_of_samples
poseData = receive(estimatedPose,10);
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