import gtsam.*

%% Assumptions
%  - All values are axis aligned
%  - Robot poses are facing along the X axis (horizontal, to the right in images)
%  - We have full odometry for measurements
%  - The robot is on a grid, moving 2 meters each step
%% Create keys for variables
i1 = symbol('x',1); i2 = symbol('x',2); i3 = symbol('x',3);
j1 = symbol('l',1); j2 = symbol('l',2); j3 = symbol('l',3);
j4 = symbol('l',4);
%% Create graph container and add factors to it
graph = NonlinearFactorGraph;

%% Add prior
priorMean = Pose2(0.0, 0.0, 0.0); % prior at origin
priorNoise = noiseModel.Diagonal.Sigmas([0.3; 0.3; 0.1]);
graph.add(PriorFactorPose2(i1, priorMean, priorNoise));


%% Add bearing/range measurement factors
degrees = pi/180;
brNoise = noiseModel.Diagonal.Sigmas([0.01]);
noiseModels.range = noiseModel.Isotropic.Sigma(1, 0.01);

GPS_1 = Pose2(0.5, 0.866, 0.0);
GPS_2 = Pose2(0.5, -0.866, 0.0);
GPS_3 = Pose2(-1.0, 0.0, 0.0);
GPS_Noise = noiseModel.Diagonal.Sigmas([10; 10; 10]);

graph.add(BetweenFactorPose2(i1, i2, GPS_1, GPS_Noise));
graph.add(BetweenFactorPose2(i2, i3, GPS_2, GPS_Noise));
graph.add(BetweenFactorPose2(i3, i1, GPS_3, GPS_Noise));

graph.add(RangeFactorPosePoint2(i1, j1, 0, noiseModels.range));
graph.add(RangeFactorPosePoint2(i2, j2, 0, noiseModels.range));
graph.add(RangeFactorPosePoint2(i3, j3, 0, noiseModels.range));

graph.add(BearingFactor2D(i1, j2, Rot2( pi/3), brNoise));
graph.add(BearingFactor2D(i2, j1, Rot2(4*pi/3), brNoise));

graph.add(BearingFactor2D(i1, j3, Rot2(0), brNoise));
graph.add(BearingFactor2D(i3, j1, Rot2(pi), brNoise));

graph.add(BearingFactor2D(i2, j3, Rot2(5*pi/3), brNoise));
graph.add(BearingFactor2D(i3, j2, Rot2(2*pi/3), brNoise));

graph.add(BearingFactor2D(i2, j4, Rot2(0), brNoise));
graph.add(BearingFactor2D(i3, j4, Rot2(pi/3), brNoise));

% print
graph.print(sprintf('\nFull graph:\n'));

%% Add pose constraint
%graph.add(BetweenFactorPose2(5, 2, Pose2(2, 0, pi/2), model));

% print
graph.print(sprintf('\nFactor graph:\n'));

%% Initialize to noisy points
initialEstimate = Values;
initialEstimate.insert(i1, Pose2(0.0, 0.0,  0.0));
initialEstimate.insert(i2, Pose2(.5, 1.866,  0.0));
initialEstimate.insert(i3, Pose2(1.0, 0.0,  0.0));

initialEstimate.insert(j1, Point2(0.0, 0.0));
initialEstimate.insert(j2, Point2(.5, .866));
initialEstimate.insert(j3, Point2(1.0, 0.0));
initialEstimate.insert(j4, Point2(6, 2));

initialEstimate.print(sprintf('\nInitial estimate:\n'));

%% Optimize using Levenberg-Marquardt optimization with an ordering from colamd
optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate);
result = optimizer.optimizeSafely();
result.print(sprintf('\nFinal result:\n'));

%% Plot Covariance Ellipses
cla;hold on

marginals = Marginals(graph, result);
plot2DTrajectory(result, [], marginals);
plot2DPoints(result, 'b', marginals);

% plot([result.at(i1).x; result.at(j1).x],[result.at(i1).y; result.at(j1).y], 'c-');
% plot([result.at(i2).x; result.at(j1).x],[result.at(i2).y; result.at(j1).y], 'c-');
% plot([result.at(i3).x; result.at(j2).x],[result.at(i3).y; result.at(j2).y], 'c-');
axis([-0.6 4.8 -1 1])
axis equal
view(2)

% cla;
% hold on
% plot([result.at(5).x;result.at(2).x],[result.at(5).y;result.at(2).y],'r-');
% marginals = Marginals(graph, result);
% 
% plot2DTrajectory(result, [], marginals);
% for i=1:5,marginals.marginalCovariance(i),end
% axis equal
% axis tight
% view(2)
