close all
clear
clc

#load dependencies
addpath "../../"
addpath "../tools/g2o_wrapper"
addpath "../tools/visualization"
source "../tools/utilities/geometry_helpers_3d.m"

#load your own dataset dataset, without landmarks (first entry remains empty)
[landmarks, poses, transitions, observations,params_offset] = loadG2o("../dataset/kalman_based_imu_slam.g2o");
correction_offset = params_offset(1);
prediction_offset = params_offset(2);
#set initial pose at the origin - we don't know the map and neither our location
#mu = [0;  #x coordinate
#      0;  #y coordinate
#      0]; #orientation theta (yaw angle)
#printf("initial pose: [%f, %f, %f]\n", mu(1), mu(2), mu(3));
mu = [poses(1).x;poses(1).y;poses(1).z;poses(1).phi;poses(1).theta;poses(1).psi];
printf("Initial pose : %f, %f, %f, %f, %f, %f, %f \n", mu(1), mu(2), mu(3),mu(4),mu(5),mu(6));


#initialize covariance: high value means high uncertainty
#sigma = eye(3);
sigma = inv(transitions(1).information);

#bookkeeping: to and from mapping between robot pose (x,y, theta) and landmark indices (i)
#all mappings are initialized with invalid value -1 (meaning that the index is not mapped)
#since we do not know how many landmarks we will observe, we allocate a large enough buffer
id_to_state_map = ones(10000, 1)*-1;
state_to_id_map = ones(10000, 1)*-1;
#reobserved = ones(1000, 1)*0

#initialize GUI with initial situation
figure("name", "ekf_slam",    #figure title
       "numbertitle", "off"); #remove figure number
trajectory = [mu(1), mu(2)];
#mu_plot = [mu(1), mu(2), mu(6)]
#plotStateEKFSLAM(mu, sigma, [], state_to_id_map, trajectory);
last_landmark_id = 1
#simulation cycle: for the number of transitions recorded in the dataset
for t = 1:length(transitions)

  if(t <= 88) 

    #obtain current transition
    transition = transitions(t);
    
    #obtain current observation
    #observation = observations(t);

    #EKF predict
    #[mu, sigma] = prediction(mu, sigma, transition);
    [mu, sigma] = prediction(mu, sigma, transition,prediction_offset);
    #disp("mu predic")
    #disp(mu(3))
    
    observation = data_association(mu, sigma, observations(t), state_to_id_map);
    #disp(observation)

    #EKF correct
    [mu, sigma, id_to_state_map, state_to_id_map,last_landmark_id] = correction(mu, 
                                                              sigma, observation, 
                                                              id_to_state_map, 
                                                              state_to_id_map,
                                                              correction_offset,
                                                              last_landmark_id);
    #disp("mu correct")
    #disp(mu(3))

    #disp(sigma)
    disp("#corrected");
		disp(mu(1));
    disp(mu(2));
    disp(mu(3));
		disp(mu(4));
    disp(mu(5));
    disp(mu(6));
		#disp("#");
    disp("truth")
		disp(poses(t));
		disp("#");

    #display current state and wait briefly
    #printf("current pose: [%f, %f, %f]\n", mu(1), mu(2), mu(3));
    #trajectory = [trajectory; mu(1), mu(2)];
    trajectory = [trajectory;poses(t).x, poses(t).y]
    #mu_plot = [mu(1), mu(2), mu(6)]
    #disp(observation)
    R_offset = euler2Rot(correction_offset.phi, correction_offset.theta, correction_offset.psi);
    v_offset = [correction_offset.x, correction_offset.y, correction_offset.z]';
    plotStateEKFSLAM(mu, sigma, observation, state_to_id_map, trajectory,R_offset,v_offset);
    pause(.1)
    fflush(stdout);	
  endif
endfor
#disp("REOBBBB")
#disp(reobserved)
#filename = "reobserved.txt";
#fid = fopen (filename, "w");
#fwrite (fid, reobserved);
#fclose (fid);
pause(10000)

