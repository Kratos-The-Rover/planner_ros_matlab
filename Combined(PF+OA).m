rosshutdown
rosinit("192.168.43.116",11311);
path = [2.00    1.00;
        1.25    1.75;
        5.25    8.25;
        7.25    8.75;
        11.75   10.75;
        12.00   10.00];
robotInitialLocation = path(1,:);
robotGoal = path(end,:);
initialOrientation = 0;
robotCurrentPose = [robotInitialLocation initialOrientation];
robot = differentialDriveKinematics("TrackWidth", 1, "VehicleInputs", "VehicleSpeedHeadingRate");
robot1 = rospublisher('/mobile_base/commands/velocity');
velMsg=rosmessage(robot1);
figure
plot(path(:,1), path(:,2),'k--d')
xlim([0 13])
ylim([0 13])
controller = controllerPurePursuit;
controller.Waypoints = path;
controller.DesiredLinearVelocity = 0.6;
controller.MaxAngularVelocity = 2;
controller.LookaheadDistance = 0.3;
goalRadius = 0.1;
distanceToGoal = norm(robotInitialLocation - robotGoal);
% Initialize the simulation loop
sampleTime = 0.1;
vizRate = rateControl(1/sampleTime);
% Initialize the figure
figure

% Determine vehicle frame size to most closely represent vehicle with plotTransforms
frameSize = robot.TrackWidth/0.8;
rostopic info /mobile_base/commands/velocity
laserSub = rossubscriber('/scan');
%rostopic echo /camera/rgb/image_raw
[velPub, velMsg] = rospublisher('/cmd_vel_mux/input/teleop');
%rostopic echo /cmd_vel_mux/input/teleop
vfh = controllerVFH; 
vfh.UseLidarScan = true;
vfh.DistanceLimits = [0.05 1];
vfh.RobotRadius = 0.1;
vfh.MinTurningRadius = 0.2;
vfh.SafetyDistance = 10;
vfh.TargetDirectionWeight=50;
vfh.CurrentDirectionWeight=0;
vfh.PreviousDirectionWeight=0;
targetDir = 0;
 
rate=rateControl(10);

rate.TotalElapsedTime
while( distanceToGoal > goalRadius )
    rate.TotalElapsedTime
    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = controller(robotCurrentPose);
    
	% Get laser scan data
    
	laserScan = receive(laserSub);
	ranges = double(laserScan.Ranges)
	angles = double(laserScan.readScanAngles);
    
	% Create a lidarScan object from the ranges and angles
        scan = lidarScan(ranges,angles);
        
	% Call VFH object to computer steering direction
	steerDir = vfh(scan, targetDir);  
    
	% Calculate velocities
	if ~isnan(steerDir) % If steering direction is valid
		desiredV = v;
		w = omega;
	else % Stop and search for valid direction
		desiredV = 0.0;
		w = 0.5;
	end

    velMsg.Linear.X = desiredV;
	velMsg.Angular.Z = w;
	send(robot1,velMsg);
    % Get the robots velocity using controller inputs
    vel = derivative(robot, robotCurrentPose, [desiredV w]);
    
    
    % Update the current pose
    robotCurrentPose = robotCurrentPose + vel*sampleTime; 
    
    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
    
    % Update the plot
    hold off
    
    % Plot path each instance so that it stays persistent while robot mesh
    % moves
    plot(path(:,1), path(:,2),"k--d")
    hold all
    
    % Plot the path of the robot as a set of transforms
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec, plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
    light;
    xlim([0 13])
    ylim([0 13])
    
    waitfor(vizRate);
end
