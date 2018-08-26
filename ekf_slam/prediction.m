#this function implements the kalman prediction step of our SLAM system
# inputs: 
#   transition: is a data structure containing several things,
#               coming from the simulator
#               of these we are interested only in the offset
#               accessible through the "v" field of the structure
#               
#               transition.v(1)=offset x
#               transition.v(2)=offset y (ignore for a diff drive robot)
#               transition.v(3)=offset theta 
#
#  considering as 'm' the number of seen landmarks
#  mu: is the mean of (x,y,theta, l1, l2, ..., lm), i.e. the previously estimated robot pose
#      and the m landmark positions
#  sigma: is the mean of the previously estimated robot pose and landmark positions ((3+m)x(3+m) matrix)

# outputs 
# [mu, sigma] are mean and covariance of the estimate after transition

function [mu, sigma] = prediction(mu, sigma, transition,prediction_offset)

%Applying the offset
#disp(transition)
pose = transition.pose;

v_offset = [prediction_offset.x,prediction_offset.y,prediction_offset.z, prediction_offset.phi, prediction_offset.theta, prediction_offset.psi]';

%T_offset is Sensor seen from R

T_offset = v2t(v_offset);
R_offset = T_offset(1:3,1:3);

v_predict = [pose.x, pose.y, pose.z, pose.phi, pose.theta, pose.psi]';

T_predict = v2t(v_predict);

T_true = T_offset * T_predict;

%Coovariance's offset

coov = inv(transition.information);

coov(1:3,1:3) = R_offset * coov(1:3,1:3) * R_offset';
#coov(1:4,1:4) = T_offset * coov(1:4,1:4) * T_offset';

%Measures offset

mu(1:6) = t2v(T_true);
#normalized_angles = [normalize_angle(mu(4));normalize_angle(mu(5));normalize_angle(mu(6))];
#mu(4:6) = normalized_angles;
sigma(1:6,1:6) = coov;
end
