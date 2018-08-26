function out = drawHighlObservations(pose,observations,R_off,v_off)

	N = length(observations.observation);
	l = 1.5;

	if N > 0
		mode = 'points';
		if isfield(observations.observation(1),'bearing')
			mode = 'bearing';
		end
		hold on;
		for i=1:N
			current_observation = observations.observation(i);
			new_pos = R_off * [current_observation.x_pose; current_observation.y_pose; current_observation.z_pose]  + v_off;
			current_observation.x_pose = new_pos(1)
			current_observation.y_pose = new_pos(2)
			current_observation.z_pose = new_pos(3)
			
			phi = pose(4);
			theta = pose(5);
			psi = pose(6);
			R = euler2Rot(phi,theta,psi);
			#mu_pos = [pose(1),pose(2),pose(3)]
			#land_abs_pose = mu_pos + R *  new_pos;
			if strcmp(mode, 'points')
				land_obs = [current_observation.x_pose; current_observation.y_pose; 0];		
				#disp("land_obs")
				#disp(land_obs)	
				#disp("rob pos")
				#disp(pose)
				#
				#pose = [pose(1),pose(2),0,0,0,pose(3)]
				land_obs = [land_obs(1),land_obs(2),land_obs(3),0,0,0]
				#
				land_abs_pose = t2v(v2t(pose)*v2t(land_obs));
				current_observation.x_pose = land_abs_pose(1);
				current_observation.y_pose = land_abs_pose(2);
				drawLandmarks(current_observation,'b','draw');
			else
				land_obs = current_observation.bearing;
				incr_y = sin(land_obs)*l;
				incr_x = cos(land_obs)*l;
				%manifold stuff
				ray_x = pose(1) + incr_x*cos(pose(3)) - incr_y*sin(pose(3));
				ray_y = pose(2) + incr_y*cos(pose(3)) + incr_x*sin(pose(3));
				hold on;
				plot([pose(1) ray_x] , [pose(2) ray_y], 'b', 'linewidth', 1.5);
			end
		end		

	end
end
