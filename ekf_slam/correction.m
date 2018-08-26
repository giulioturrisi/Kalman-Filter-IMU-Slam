#this function computes the update (also called correction)
#step of the filter
#inputs:
#  mu: mean, 
#  sigma: covariance of the robot-landmark set (x,y, theta, l_1, ..., l_N)
#
#  observations:
#            a structure containing n observations of landmarks
#            for each observation we have
#            - the index of the landmark seen
#            - the location where we have seen the landmark (x,y) w.r.t the robot
#
#  id_to_state_map:
#            mapping that given the id of the measurement, returns its position in the mu vector
#  state_to_id_map:
#            mapping that given the index of mu vector, returns the id of the respective landmark
#
#outputs:
#  [mu, sigma]: the updated mean and covariance
#  [id_to_state_map, state_to_id_map]: the updated mapping vector between landmark position in mu vector and its id

function [mu, sigma, id_to_state_map, state_to_id_map,last_landmark_id] = correction(mu, sigma, observations, id_to_state_map, state_to_id_map, correction_offset,last_landmark_id)

  #determine how many landmarks we have seen in this step
  num_landmarks_measured = length(observations.observation);
  
  #dimension of the state (robot pose + landmark positions)
  state_dim = size(mu,1);

  #dimension of the current map (how many landmarks we have already seen)
  num_landmarks = (state_dim-6)/3;

  #if I've seen no landmarks, i do nothing
  disp(num_landmarks_measured)
  if (num_landmarks_measured == 0)
    disp("esco")
    return;
  endif

  %Offset calculations for measurements
  R_offset = euler2Rot(correction_offset.phi, correction_offset.theta, correction_offset.psi);
  v_offset = [correction_offset.x, correction_offset.y, correction_offset.z]';

  %Sigma of the observations
  sigma_observation = inv(observations.information);
  sigma_observation = R_offset * sigma_observation * R_offset';

  mu_pos = mu(1:3,:); # translational part of the robot pose
  mu_rot = [mu(4,:), mu(5,:), mu(6,:)];
  #mu_theta = mu(3); # rotation of the robot

  # re precompute some quantities that come in handy later on
  #c   = cos(mu_theta);
  #s   = sin(mu_theta);
  #R   = [c -s; s c];  #rotation matrix
  #Rt  = [c,s;-s c];    #transposed rotation matrix
  #Rtp = [-s,c;-c,-s]; #derivative of transposed rotation matrix
  phi = mu_rot(1);
  theta = mu_rot(2);
  psi = mu_rot(3);
  dRx = Rx_prime(phi);
  dRy = Ry_prime(theta);
  dRz = Rz_prime(psi);
  R = euler2Rot(phi,theta,psi);
  Rt = R';

  # for below computation, we need to count how many observations 
  # of old landmarks we have
  num_old_landmarks_measured = 0;

  # Here two cases arise, the current landmark has been already seen, i.e. REOBSERVED landmark,
  # or the current landmark is completely new, i.e. NEW landmark.
  #
  # for simplicity we can divide the problem: first analyze only the reobserved landmark
  # and work with them as in a localization procedure (of course, with full Jacobian now).
  # With this reobserved landmark we compute the correction/update of mean and covariance.
  # Then, after the correction is done, we can simply add the new landmark expanding the
  # mean and the covariance.
  #
  #
  # First of all we are interested in REOBSERVED landmark 
  for i=1:num_landmarks_measured
  
    #retrieve info about the observed landmark
    measurement = observations.observation(i);

    if (measurement.id < 1)
	    continue;
    endif

    #fetch the position in the state vector corresponding to the actual measurement
    state_pos_of_landmark = id_to_state_map(measurement.id);

    #compute the index (vector coordinate) in the state vector corresponding to the pose of the landmark;	
    landmark_state_vector_index = 7+3*(state_pos_of_landmark-1);

    #IF current landmark is a REOBSERVED LANDMARK
    if(state_pos_of_landmark != -1) 
      disp("REAOBSERVEEED")
      disp(measurement.id)
      #reobserved(measurement.id) = 2

      #increment the counter of observations originating
      #from already known landmarks
      num_old_landmarks_measured++;
      measurement_true = R_offset * [measurement.x_pose; measurement.y_pose; measurement.z_pose]  + v_offset;

      # where we see the landmark
      z_t(end+1,:) = measurement_true(1); 
      z_t(end+1,:) = measurement_true(2);
      z_t(end+1,:) = measurement_true(3);

      #fetch the position of the landmark in the state (its x and y coordinate)
      landmark_mu=mu(landmark_state_vector_index:landmark_state_vector_index+2,:);
      
      #where I predict i will see that landmark
      delta_t            = landmark_mu-mu_pos;
      measure_prediction = Rt* delta_t;

      #add prediction to prediction vector
      h_t(end+1,:) = measure_prediction(1);
      h_t(end+1,:) = measure_prediction(2);
      h_t(end+1,:) = measure_prediction(3);

      #jacobian w.r.t robot
      C_m=zeros(3,state_dim);
      C_m(1:3,1:3) = -Rt;
      C_m(1:3,4) = (Rz(psi) * Ry(theta) * dRx)' * delta_t;
      C_m(1:3,5) = (Rz(psi) * dRy   * Rx(phi))' * delta_t;
      C_m(1:3,6) = (dRz * Ry(theta) * Rx(phi))' * delta_t;

      #jacobian w.r.t landmark
      C_m(:,landmark_state_vector_index:landmark_state_vector_index+2)=Rt;
      C_t(end+1,:) = C_m(1,:);
      C_t(end+1,:) = C_m(2,:);
      C_t(end+1,:) = C_m(3,:);
    endif
  endfor

  #if I have seen again at least one landmark
  #I need to update, otherwise I jump to the new landmark case
  if ((num_old_landmarks_measured > 0))

    #observation noise
    noise   = 0.01;
    sigma_z = eye(3*num_old_landmarks_measured)*noise;

    #Kalman gain
    K = sigma * C_t'*(inv(sigma_z + C_t*sigma*C_t'));

    #update mu
    error      = (z_t - h_t);
    correction = K*error;
    #disp("CORRECTION")
    #disp(correction)
    mu         = mu + correction;

    #update sigma
    sigma = (eye(state_dim) - K*C_t)*sigma;		
    #disp(sigma)
  endif

  #since I have applied the correction, I need to update my
  #data with the new mu values
  #mu_t     = mu(1:2,:); #translational part of the robot pose
  #mu_theta = mu(3);     #rotation of the robot
  mu_pos = mu(1:3,:); # translational part of the robot pose
  mu_rot = [mu(4,:), mu(5,:), mu(6,:)];

  #re precompute some quantities that come in handy later on
  #c   = cos(mu_theta);
  #s   = sin(mu_theta);
  #R   = [c -s; s c];  #rotation matrix
  #Rt  = [c,s;-s c];    #transposed rotation matrix
  #Rtp = [-s,c;-c,-s]; #derivative of transposed rotation matrix
  phi = mu_rot(1);
  theta = mu_rot(2);
  psi = mu_rot(3);
  dRx = Rx_prime(phi);
  dRy = Ry_prime(theta);
  dRz = Rz_prime(psi);
  R = euler2Rot(phi,theta,psi);
  Rt = R';

  #Now its time to add, if observed, the NEW landmaks, without applying any correction
  for i=1:num_landmarks_measured

    #retrieve info about the observed landmark
    measurement = observations.observation(i);

    if (measurement.id == 0)
	    continue;
    elseif(measurement.id == -1)
	    measurement.id = last_landmark_id++; %new landmark
    endif

    #fetch the position in the state vector corresponding to the actual measurement
    state_pos_of_landmark=id_to_state_map(measurement.id);

    #IF current landmark is a NEW landmark
    if(state_pos_of_landmark == -1) 

      #adjust direct and reverse mappings
      num_landmarks++;
      id_to_state_map(measurement.id)=num_landmarks;
      #disp("num_landmarks")
      #disp(num_landmarks)
      state_to_id_map(num_landmarks)=measurement.id;

      measurement_true = R_offset * [measurement.x_pose; measurement.y_pose; measurement.z_pose]  + v_offset;
      
      #landmark position in the world
      #land_obs = [measurement_true(1),measurement_true(2),measurement_true(3),0,0,0]
      #land_pose_world = t2v(v2t(mu(1:6))*v2t(land_obs));
      land_pose_world = mu_pos + R *  measurement_true;

      #retrieve from the index the position of the landmark block in the state
      new_landmark_state_vector_index=7+3*(num_landmarks-1);
 
      #increase mu and sigma size
      #disp("mu before")
      #disp(mu)
      mu(new_landmark_state_vector_index:new_landmark_state_vector_index+2,1) = land_pose_world;
      #disp("mu after")
      #disp(mu)
      #initial noise assigned to a new landmark
      #for simplicity we put a high value only in the diagonal.
      #A more deeper analysis on the initial noise should be made.
      initial_landmark_noise=1;
      landmark_sigma = eye(3)*initial_landmark_noise;

      #extend the structure
      #disp("sigma before")
      #disp(sigma)
      sigma(new_landmark_state_vector_index,:)   = 0;
      sigma(new_landmark_state_vector_index+2,:) = 0;
      sigma(:,new_landmark_state_vector_index)   = 0;
      sigma(:,new_landmark_state_vector_index+2) = 0;

      #add the covariance block
      sigma(new_landmark_state_vector_index:new_landmark_state_vector_index+2,
	    new_landmark_state_vector_index:new_landmark_state_vector_index+2)=landmark_sigma;
      #disp("sigma after")
      #disp(sigma)

      printf("observed new landmark with identifier: %i \n",measurement.id);
      fflush(stdout);
    endif
  endfor
end

