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
priorMeanPose = Pose2(0.0, 0.0, 0.0); % prior at origin
priorMeanPoint = Point2(0.0, 0.0); % prior at origin
priorNoisePose = noiseModel.Diagonal.Sigmas([0.3; 0.3; 1000]);
priorNoisePoint = noiseModel.Diagonal.Sigmas([0.3; 0.3]);

brNoise = noiseModel.Diagonal.Sigmas([0.01]);
noiseModels.range = noiseModel.Isotropic.Sigma(1, 0.01);

GPS_Noise = noiseModel.Isotropic.Sigma(1, 1);

initialEstimate = Values;

%% Load image data
our_path = 'Photos\';
folder = 'BaseBall';
anglesName = 'angles.csv';
xyName = 'xy.csv';

angles = csvread(strcat(our_path, folder, '\', anglesName));
xy =     csvread(strcat(our_path, folder, '\', xyName));


%% Build Factor Graph
for i = 1:size(angles,2)        %#ofcolums
    %Add a pose
    poseSymbol1 = symbol('X',i);
    pointSymbol1 = symbol('O',i);
    graph.add(PriorFactorPose2(poseSymbol1, Pose2(xy(i,1), xy(i,2),  0.0), priorNoisePose)); %!!!!!!!!!!!!!!!
    graph.add(PriorFactorPoint2(pointSymbol1, Point2(xy(i,1), xy(i,2)), priorNoisePoint));
    graph.add(RangeFactorPosePoint2(poseSymbol1, pointSymbol1, 0, noiseModels.range));
    
    initialEstimate.insert(poseSymbol1, Pose2(xy(i,1), xy(i,2),  0.0));
    initialEstimate.insert(pointSymbol1, Point2(xy(i,1), xy(i,2)));
    
    %Add point
    %Initialize both
    for j = 1:size(angles,1)    %#ofrows
        angle = angles(j,i);
        if ~(isnan(angle))
            pointSymbol2= symbol('O',j);
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


% print
graph.print(sprintf('\nFactor graph:\n'));


%% Optimize using Levenberg-Marquardt optimization with an ordering from colamd
optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate);
result = optimizer.optimizeSafely();
result.print(sprintf('\nFinal result:\n'));

%% Plot Covariance Ellipses
cla;hold on

marginals = Marginals(graph, result);
plot2DTrajectory(result, [], marginals);
plot2DPoints(result, 'b', marginals);
