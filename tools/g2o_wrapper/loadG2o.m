% #   This source code is part of the localization and SLAM package
% #   deveoped for the lectures of probabilistic robotics at 
% #   Sapienza, University of Rome.
% #  
% #     Copyright (c) 2016 Bartolomeo Della Corte, Giorgio Grisetti
% #  
% #   It is licences under the Common Creative License,
% #   Attribution-NonCommercial-ShareAlike 3.0
% #  
% #   You are free:
% #     - to Share - to copy, distribute and transmit the work
% #     - to Remix - to adapt the work
% #  
% #   Under the following conditions:
% #  
% #     - Attribution. You must attribute the work in the manner specified
% #       by the author or licensor (but not in any way that suggests that
% #       they endorse you or your use of the work).
% #    
% #     - Noncommercial. You may not use this work for commercial purposes.
% #    
% #     - Share Alike. If you alter, transform, or build upon this work,
% #       you may distribute the resulting work only under the same or
% #       similar license to this one.
% #  
% #   Any of the above conditions can be waived if you get permission
% #   from the copyright holder.  Nothing in this license impairs or
% #   restricts the author's moral rights.
% #  
% #   This software is distributed in the hope that it will be useful,
% #   but WITHOUT ANY WARRANTY; without even the implied 
% #   warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
% #   PURPOSE.
% #
%   
% # load a file.g2o file and returns the four structs of landmark, poses, transitions, observations

function [landmarks, poses, transitions, observations,params_offset] = loadG2o(filepath)

	%%Load of Geometry Helper for Quaternions conversion

	


	%%-----G2O specification---
	VERTEX_XY = 'VERTEX_XY';
	VERTEX_TRACKXYZ = 'VERTEX_TRACKXYZ';
	VERTEX_SE2 = 'VERTEX_SE2';
	VERTEX_SE3_QUAT = 'VERTEX_SE3:QUAT';
	ROBOTLASER1 = 'ROBOTLASER1';
	EDGE_SE3_TRACKXYZ = 'EDGE_SE3_TRACKXYZ';
	EDGE_SE3_PRIOR = 'EDGE_SE3_PRIOR';
	EDGE_SE2 = 'EDGE_SE2';
	EDGE_BEARING_SE2_XY = 'EDGE_BEARING_SE2_XY';
	EDGE_SE2_XY = 'EDGE_SE2_XY';
	PARAMS_SE3OFFSET = 'PARAMS_SE3OFFSET';
	%%-------------------------

	%open the file
	fid = fopen(filepath, 'r');
	

	%debug stuff
	i_vert_xy = 0;
	i_vert_se2 = 0;
	i_vertex_trackxyz = 0;
	i_vertex_se3_quat = 0;
	i_robotlaser = 0;
	i_edge_se3_trackxyz = 0;
	i_edge_se3_prior = 0;
	i_edge_se2 = 0;
	i_edge_se2_xy = 0;
	i_edge_bearing_se2_xy=0;
	i_params_se3offset=0;
  
	%    
	curr_id = -1;

	while true
		%get current line
		c_line = fgetl(fid);

		%stop if EOF
		if c_line == -1
			break;
		end

		%Split the line using space as separator
		elements = strsplit(c_line,' ');

		switch(elements{1})

			%Real positions of the Landmarks
			case VERTEX_TRACKXYZ
					landmarks(end+1) = extractLandmark3d(elements);
				i_vertex_trackxyz = i_vertex_trackxyz + 1;
			%Real poses of the robot
			case VERTEX_SE3_QUAT
					poses(end+1) = extractPose_3d_Quat(elements);
				i_vertex_se3_quat = i_vertex_se3_quat + 1;
			
			%Observations made from the robot
			case EDGE_SE3_TRACKXYZ
				current_obs = extractPoint3d(elements);
				%disp('--------');
				%disp('observations ID');
				if current_obs.pose_id == curr_id
					observations(end).observation(end+1) = current_obs.observation;
				%	disp(observations(end).pose_id);
				else
					observations(end+1) = current_obs;
					curr_id = observations(end).pose_id;
				%	disp(curr_id);
					i_edge_se3_trackxyz = i_edge_se3_trackxyz + 1;
				end
			%Prediction made from the IMU
			case EDGE_SE3_PRIOR
				transitions(end+1) = extractTransition_IMU(elements);
				%disp('Transition ID');
				%disp(transitions(end).id_from);
				%disp('------');
				i_edge_se3_prior = i_edge_se3_prior + 1;
			%Sensors offset
			case PARAMS_SE3OFFSET
				params_offset(end+1) = extractOffsetSensors(elements);
				i_params_se3offset = i_params_se3offset + 1;

			otherwise
				disp('Error in reading first element');
		end
	end
  
  printf('[G2oWrapper] loading file...\n#landmarks: %d \n#poses: %d \n',i_vertex_trackxyz, i_vertex_se3_quat);
  printf('#transitions: %d \n#observation: %d \n',i_edge_se3_prior, i_edge_se3_trackxyz);  
  fflush(stdout);

end

function out = extractLandmark(elements)
  id = str2double(elements{2});
  x_pose = str2double(elements{3});
  y_pose = str2double(elements{4});
  out = landmark(id,[x_pose,y_pose]);
end

function out = extractLandmark3d(elements)
	id = str2double(elements{2});
  	x_pose = str2double(elements{3});
  	y_pose = str2double(elements{4});
  	z_pose = str2double(elements{5});
  	out = landmark(id,[x_pose,y_pose,z_pose]);
end

function out = extractPose(elements)
  id = str2double(elements{2});
  x_pose = str2double(elements{3});
  y_pose = str2double(elements{4});
  th_pose = str2double(elements{5});
  out = pose(id,x_pose, y_pose, th_pose);
end

function out = extractPose_3d_Quat(elements)
  id = str2double(elements{2});
  x_pose = str2double(elements{3});
  y_pose = str2double(elements{4});
  z_pose = str2double(elements{5});
  [phi,theta,psi] = quat2euler(str2double(elements{6}),str2double(elements{7}),str2double(elements{8}),str2double(elements{9}));
  out = pose3d(id,x_pose,y_pose,z_pose,phi,theta,psi);
end

function out = extractTransition(elements)
  from_id = str2double(elements{2});
  to_id = str2double(elements{3});
  x_t = str2double(elements{4});
  y_t = str2double(elements{5});
  th_t = str2double(elements{6});
  out = transition(from_id,to_id, [x_t;y_t;th_t]);
end

function out = extractTransition_IMU(elements)
  from_id = str2double(elements{2});
  %elements{3} is the sensor ID
  sensor_id = str2double(elements{3});
  
  x_t = str2double(elements{4});
  y_t = str2double(elements{5});
  z_t = str2double(elements{6});
  [phi_t,theta_t,psi_t] = quat2euler(str2double(elements{7}),str2double(elements{8}),str2double(elements{9}),str2double(elements{10}));
  information = [str2double(elements{11}),str2double(elements{12}),str2double(elements{13}),str2double(elements{14}),str2double(elements{15}),str2double(elements{16});
  				 str2double(elements{12}),str2double(elements{17}),str2double(elements{18}),str2double(elements{19}),str2double(elements{20}),str2double(elements{21});
  				 str2double(elements{13}),str2double(elements{18}),str2double(elements{22}),str2double(elements{23}),str2double(elements{24}),str2double(elements{25});
  				 str2double(elements{14}),str2double(elements{19}),str2double(elements{23}),str2double(elements{26}),str2double(elements{27}),str2double(elements{28});
  				 str2double(elements{15}),str2double(elements{20}),str2double(elements{24}),str2double(elements{27}),str2double(elements{29}),str2double(elements{30});
  				 str2double(elements{16}),str2double(elements{21}),str2double(elements{25}),str2double(elements{28}),str2double(elements{30}),str2double(elements{31})];
  				 
  out = transition_IMU(from_id,sensor_id,[x_t,y_t,z_t,phi_t,theta_t,psi_t],information);
end

function out = extractBearing(elements)
  from_id = str2double(elements{2});
  land_id = str2double(elements{3});
  bearing = str2double(elements{4});
  out = observation(from_id,land_id, bearing);
end

function out = extractPoint(elements)
  from_id = str2double(elements{2});
  land_id = str2double(elements{3});
  x_p = str2double(elements{4});
  y_p = str2double(elements{5});
  out = observation(from_id,land_id, [x_p; y_p]);
end

function out = extractPoint3d(elements)
  from_id = str2double(elements{2});
  land_id = str2double(elements{3});
  %elements{4} is the ID of the sensor
  x_p = str2double(elements{5});
  y_p = str2double(elements{6});
  z_p = str2double(elements{7});
  information = [str2double(elements{8}),str2double(elements{9}),str2double(elements{10});
  				 str2double(elements{9}),str2double(elements{11}),str2double(elements{12});
  				 str2double(elements{10}),str2double(elements{12}),str2double(elements{13})];
  out = observation(from_id,land_id, [x_p;y_p;z_p],information);
end

function out = extractOffsetSensors(elements)
	sensor_id  = str2double(elements{2});
	x_offset   = str2double(elements{3});
	y_offset   = str2double(elements{4});
	z_offset   = str2double(elements{5});

	[phi_offset,theta_offset,psi_offset] = quat2euler(str2double(elements{6}),str2double(elements{7}),str2double(elements{8}),str2double(elements{9}));
	out = params_offset(sensor_id, x_offset, y_offset, z_offset, phi_offset, theta_offset, psi_offset);
end

