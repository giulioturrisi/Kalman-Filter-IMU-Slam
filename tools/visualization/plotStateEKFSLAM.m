function out = plotStateEKFSLAM(mean, covariance, observations, state_to_id_map, trajectory_)

	robot_pose = mean(1:3);
	state_dim = size(mean,1);
	map_size = (state_dim - 3)/2; #number of landmark(if any)

	if(nargin == 2) %init step
		hold on;
		#axis([-12 12 -12 12]);
		drawRobot(robot_pose, zeros(3));
	else
    hold off;

    #draw robot trajectory
    drawTrajectoryXY(trajectory_);

		%draw landmarks
		if(map_size > 0)
			drawLandmarks_slam(mean(4:end), state_to_id_map);
		end

		drawRobot(robot_pose, covariance);
		if(length(observations) > 0)
			drawHighlObservations(robot_pose,observations);
		endif
		#plot robot covariance		
		plotcov2d(robot_pose(1),robot_pose(2),covariance(1:2,1:2),'k', 1);
		#plot landmark(if any) covariance
		for i=4:2:2*(map_size+1)
		  plotcov2d(mean(i,1), mean(i+1,1), covariance(i:i+1,i:i+1),'r',1);
		end

	end
	drawnow;

end
