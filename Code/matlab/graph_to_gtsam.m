clc
clear
import gtsam.*

%% Assumptions
%  - Axis are not aligned
%  - Robot poses are facing random directions
%  - Have innaccurate GPS locations
%  - Having accurate bearing information 

%% Create graph container and add factors to it
graph = NonlinearFactorGraph;

%% Noise models
priorMean = Pose2(0.0, 0.0, 0.0); % prior at origin
priorNoise = noiseModel.Diagonal.Sigmas([0.3; 0.3; 0.1]);

degrees = pi/180;
brNoise = noiseModel.Diagonal.Sigmas([0.01]);
noiseModels.range = noiseModel.Isotropic.Sigma(1, 0.01);

GPS_Noise = noiseModel.Isotropic.Sigma(1, 1);

initialEstimate = Values;

%% Load image data
our_path = '\\ad.gatech.edu\ecefs$\users\students\mobrien36\Profile\Desktop\Photos2\Photos\';
folder = 'BaseBall';
AngleName = strcat(folder, 'Angle.csv');
xyName = strcat(folder, 'xy.csv');

angles = csvread(strcat(our_path, folder, '\', AngleName));
xy = csvread(strcat(our_path, folder, '\', xyName));


%% Build Factor Graph
for i = 1:size(angles,2)        %#ofcolums
    %Add a pose
    poseSymbol1 = symbol('x',i);
    pointSymbol1 = symbol('l',i);
    graph.add(PriorFactorPose2(poseSymbol1, priorMean, priorNoise)); %!!!!!!!!!!!!!!!
    graph.add(PriorFactorPoint2(pointSymbol1, priorMean, priorNoise));
    graph.add(RangeFactorPosePoint2(poseSymbol1, pointSymbol1, 0, noiseModels.range));
    
    %initialEstimate.insert(poseSymbol1, Pose2(xy(i,1), xy(i,2),  0.0));
    %initialEstimate.insert(pointSymbol1, Point2(xy(i,1), xy(i,2)));
    
    %Add point
    %Initialize both
    for j = 1:size(angles,1)    %#ofrows
        angle = angles(j,i);
        if ~(isnan(angle))
            pointSymbol2= symbol('l',j)
            graph.add(BearingFactor2D(poseSymbol1, pointSymbol2, Rot2(angle), brNoise));

        end
    end
    
    
%     for j = 1:size(angles,2)    %#ofrows
%         if ~(isnan(angles(j,i)))
%             poseSymbol2 = symbol('x',j);
%             
%             xoffset = xy(j,1) - xy(i,1);
%             yoffset = xy(j,2) - xy(i,2);
%             offset = Pose2(xoffset,yoffset,0);
%             
%             graph.add(BetweenFactorPose2(poseSymbol1, poseSymbol2, offset, GPS_Noise));
%         end
%     end
end



graph.print(sprintf('\nFactor graph:\n'));
    
