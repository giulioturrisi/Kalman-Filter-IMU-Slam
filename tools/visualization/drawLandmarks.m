function done = drawLandmarks(land,color, mode)
	
	if(nargin == 1)
		color = 'r';
		mode = 'fill';
	end


	N = length(land);
	radius = 0.1;
	for i=1:N
		drawShape('circle', [land(i).x_pose, land(i).y_pose, radius], mode, color)
		hold on;
		drawLabels(land(i).x_pose, land(i).y_pose, land(i).id, '%d');
		hold on;
	end


end
