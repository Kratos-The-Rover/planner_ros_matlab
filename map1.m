% load exampleMaps
% map = binaryOccupancyMap(simpleMap);
% figure
% show(map)
%Create a 10m x 10m empty map.
robot = differentialDriveKinematics("TrackWidth", 1, "VehicleInputs", "VehicleSpeedHeadingRate");
map = binaryOccupancyMap(25,25,25);
controller = controllerPurePursuit;
sampleTime = 0.1;
vizRate = rateControl(1/sampleTime);
%Set occupancy of world locations and show map.
frameSize = robot.TrackWidth/0.8;
x = [15.2; 12.3; 11.0; 17.5; 5.6];
y = [10.0; 24.0; 9.6; 12.0; 11.0];

setOccupancy(map, [x y], ones(5,1))
inflate(map, 0.5)
figure
show(map)
mapInflated = copy(map);
inflate(mapInflated, robot.TrackWidth/2);
prm = robotics.PRM(mapInflated);
prm.NumNodes = 100;
prm.ConnectionDistance = 10;
startLocation = [4.0 2.0];
endLocation = [24.0 20.0]
path = findpath(prm, startLocation, endLocation)
show(prm);
release(controller);
controller.Waypoints = path;
controller.DesiredLinearVelocity = 10;
controller.MaxAngularVelocity = 10;
robotInitialLocation = path(1,:);
robotGoal = path(end,:);
initialOrientation = 0;
robotCurrentPose = [robotInitialLocation initialOrientation]';
h=robotCurrentPose(1:2)
startLocation=h'

%Compute distance to the goal location

distanceToGoal = norm(robotInitialLocation - robotGoal);

%Define a goal radius

goalRadius = 0.1;

%Drive the robot using the controller output on the given map until it reaches the goal. The controller runs at 10 Hz.

reset(vizRate);

% Initialize the figure
figure

while( 1>0 )
    x = [15.2; 12.3; 11.0; 17.5; 5.6];
    y = [10.0; 24.0; 9.6; 12.0; 11.0];
    t=rand(1).*10   
    x=t';
    q=rand(1).*10;
    y=q';
    setOccupancy(map, [x y], ones(5,1))
    %inflate(map, 0.1)
    figure
    show(map)
    mapInflated = copy(map)
    prm = robotics.PRM(mapInflated)
    h=robotCurrentPose(1:2)
    path = findpath(prm, h', endLocation)
    show(prm);
    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = controller(robotCurrentPose);
    
    % Get the robot's velocity using controller inputs
    vel = derivative(robot, robotCurrentPose, [v omega]);
    
    % Update the current pose
     robotCurrentPose = robotCurrentPose + vel*sampleTime; 
   
    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
    
    % Update the plot
    hold off
    show(map);
    hold all

    % Plot path each instance so that it stays persistent while robot mesh
    % moves
    plot(path(:,1), path(:,2),"k--d")
    
    % Plot the path of the robot as a set of transforms
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, 'MeshFilePath', 'groundvehicle.stl', 'Parent', gca, "View","2D", "FrameSize", frameSize);
    light;
    xlim([0 27])
    ylim([0 26])
    
    waitfor(vizRate);
end