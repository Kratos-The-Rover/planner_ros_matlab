rosshutdown
rosinit("192.168.137.157",11311);
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

targetDir = 0;

rate=rateControl(10);

rate.TotalElapsedTime
while 1 > 0
    rate.TotalElapsedTime
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
		desiredV = 0.2;
		w = exampleHelperComputeAngularVelocity(steerDir, 1);
	else % Stop and search for valid direction
		desiredV = 0.0;
		w = 0.5;
	end

	% Assign and send velocity commands
	velMsg.Linear.X = desiredV;
	velMsg.Angular.Z = w;
	velPub.send(velMsg);
    %rostopic echo /cmd_vel_mux/input/teleop
end