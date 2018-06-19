function out = plot_state(landmarks, pose, covariance, observations)

	robot_pose = pose(1:3);
	state_dim = size(pose,1);
	map_size = (state_dim - 3)/2; #number of landmark(if any)

	if(nargin == 2) %init step
		hold on;
		#axis([-12 12 -12 12]);
		drawLandmarks(landmarks);
		drawRobot(robot_pose, zeros(3));
	else
		hold off;
		
		drawLandmarks(landmarks);

		drawRobot(robot_pose, covariance);
		drawHighlObservations(robot_pose,observations);

		#plot robot covariance		
		plotcov2d(robot_pose(1),robot_pose(2),covariance(1:2,1:2),'k', 1);
		#plot landmark(if any) covariance
		for i=4:2:2*(map_size+1)
		  plotcov2d(pose(i,1), pose(i+1,1), covariance(i:i+1,i:i+1),'r',1);
		end

	end
	drawnow;

end
