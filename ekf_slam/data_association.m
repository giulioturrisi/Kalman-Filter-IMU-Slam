# perform the data associations
# 
# inputs:
#   mu: current mean
#   sigma: current covariance
#   observations: current observation set, from G2oWrapper
#   state_to_id_map: mapping vector state_position-id
#   heuristics: may be used to input the desired level of heuristics
#
# outputs:
#   observations: with the respective, aka associated, ids

function observations = data_association(mu, sigma, observations, state_to_id_map)

	disp("QUIIII")
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

	measures = observations.observation;

	# determine how many landmarks we have seen in this step
	num_landmarks_measured = length(measures);

	#set all the observations to unknown
	for i=1:num_landmarks_measured
		observations.observation(i).id = -1;
	endfor

	# dimension of the state (robot pose + landmark positions)
	state_dim = size(mu,1);

	# dimension of the current map (how many landmarks we have already seen)
	num_landmarks = (state_dim-6)/3;
    disp("num_land")
	disp(num_landmarks)
	#if 0 observations are made or current map is empty, return
	if(num_landmarks == 0 || num_landmarks_measured == 0)
		disp("qui")
		return;
	endif

	# mu_landmark part
	mu_landmarks = mu(7:end);
	# mu_robot part
	mu_robot = mu(1:6);
	# current_land_id in mu
	curr_land_id = 1;

	#build the association matrix [(current observations) x (number of landmarks)]
	A = ones(num_landmarks_measured, num_landmarks)*1e3;
	
	#now we have to populate the association matrix
	for n=1:num_landmarks

		#extract landmark from mu
		mu_curr_landmark = mu_landmarks(curr_land_id:curr_land_id+2);
		disp("mu lan")
		disp(mu_curr_landmark)
	
		#compute measure function
		delta_t            = mu_curr_landmark-mu_pos;
        h = Rt* delta_t;
		#disp("h")
		#disp(h)
		#disp("rt")
		#disp(Rt)
		C_m=zeros(3,state_dim);
		C_m(1:3,1:3) = -Rt;
		C_m(1:3,4) = (Rz(psi) * Ry(theta) * dRx)' * delta_t;
		C_m(1:3,5) = (Rz(psi) * dRy   * Rx(phi))' * delta_t;
		C_m(1:3,6) = (dRz * Ry(theta) * Rx(phi))' * delta_t;
		#jacobian w.r.t landmark
		#disp("curr_land_id")
		#disp(curr_land_id)
		C_m(:,curr_land_id:curr_land_id+2)=Rt;
		#C(end+1,:) = C_m(1,:);
		#C(end+1,:) = C_m(2,:);
		#C(end+1,:) = C_m(3,:);		

		#[h, C] = measurement_function(state_dim,mu_robot, mu_curr_landmark, curr_land_id);
		sigma_zx = eye(3,3)*0.01;
		#disp("C")
		#disp(C_m)
		sigma_nn = C_m*sigma*C_m' + sigma_zx;
		omega_nn = inv(sigma_nn);

		for m=1:num_landmarks_measured
			measure = measures(m);

			#current measurement
			z = [measure.x_pose; measure.y_pose;measure.z_pose];

			#look at the indices [(current observations) x (number of landmarks)]
			A(m,n) = (z - h)' * omega_nn * (z - h);
		endfor

		curr_land_id += 3;
	endfor #num_landmarks

	#disp("A_association")
	#disp(A)

	
	#now associate the measurement to the most promising landmark
	# proposed associations will be stored in a [Mx3] matrix composed
	# in this way
	#
	#	[ number of measure , proposed landmark id , association matrix value] 	
	#
	# we will populate such a matrix with the associations surviving
	# the gating heuristic.
	# 
	# In the best friends and lonely best friend heuristics we will keep
	# the association as is in case of success, otherwise we will put
	# an id=0 to that measurment, meaning that the association is doubtful

	## Heuristics
	tau_accept = 2.5;

	# gating
	survived = 0
	for m=1:num_landmarks_measured
			#disp("ENTROOOOOOOOOOOOOOOOOOOO??")
			#return the min and index on the 'm-th' row
			[min_arg, min_index] = min(A(m,:));

			if(min_arg < tau_accept)			
				# add the possible association
				survived++
				#disp("ENTROOOOOOOOOOOOOOOOOOOO")
				associations(end+1,:) = [m, min_index, min_arg];
			endif
	endfor

	if(survived <= 0)
		return
	endif

	#associations survied to gating
	dim_gated_associations = size(associations,1);

	#best friends
	for i=1:dim_gated_associations
		current_associations_value = associations(i,3);
		proposed_landmark = associations(i,2);
		min_on_column = min(A(:, proposed_landmark));
		if(current_associations_value != min_on_column)
			associations(i,2) = 0; #discard association, it is doubtful
		endif
	endfor

	
	#lonely best friend
	if(num_landmarks_measured > 1)
		gamma_threshold = 1e-3;

		for i=1:dim_gated_associations
			current_associations_value = associations(i,3);
			measure_number = associations(i,1);
			proposed_landmark = associations(i,2);
			if(proposed_landmark == 0)
				continue; #this association is doubtful, continue
			endif

			#obtain second best(aka min) value of the row
			ordered_row = unique(A(measure_number,:));
			second_row_min_value = ordered_row(2);
			#obtain second best(aka min) value of the column
			ordered_col = unique(A(:,proposed_landmark));
			second_col_min_value = ordered_col(2);

			#check if the association is ambiguous
			if( (second_row_min_value - current_associations_value) < gamma_threshold ||
		 	    (second_col_min_value - current_associations_value) < gamma_threshold )
				associations(i,2) = 0; #discard association, it is doubtful
			endif
	
		endfor
	endif

	#assign the associations to the observations
	for i=1:dim_gated_associations
		measure_number = associations(i,1);
		proposed_landmark = associations(i,2);	
		#disp("measure_numb")
		disp(measure_number)
		#disp("proposed_landmark")
		disp(proposed_landmark)	

		##assign the proposed association OR a 0 value that means doubt
		observations.observation(measure_number).id = proposed_landmark;
	endfor
end
