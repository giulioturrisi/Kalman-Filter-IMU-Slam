function out = plotStateEKFSLAM(mean, covariance, observations, state_to_id_map, trajectory_,R_off,v_off)

	robot_pose = mean(1:6);
	robot_pose_obs = mean(1:6);
	robot_pose = [robot_pose(1),robot_pose(2),robot_pose(6)]
	state_dim = size(mean,1);
	#disp("statedim")
	#disp(state_dim)
	map_size = (state_dim - 6)/3; #number of landmark(if any)
	#disp("mapsize")
	#disp(map_size)
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
			#drawLandmarks_slam(mean(4:end), state_to_id_map);
			#disp("land")
			#disp(mean(4:end))
			drawLandmarks_slam(mean(7:end), state_to_id_map);
		end

		drawRobot(robot_pose, covariance);
		if(length(observations) > 0)
			#disp(observations)
			drawHighlObservations(robot_pose_obs,observations,R_off,v_off);
		endif
		#plot robot covariance		
		plotcov2d(robot_pose(1),robot_pose(2),covariance(1:2,1:2),'k', 1);
		#plot landmark(if any) covariance
		#for i=4:2:2*(map_size+1)
		N = size(mean(7:end),1)/3;
		for i=1:3:3*N
		  #disp("III_land")
		  #disp(i)
		  plotcov2d(mean(i + 6,1), mean(i+1 + 6,1), covariance(i+ 6:i+1+ 6,i+ 6:i+1+ 6),'g',1);
		end

	end
	drawnow;

end
