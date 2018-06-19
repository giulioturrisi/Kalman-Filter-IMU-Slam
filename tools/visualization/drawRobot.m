function out = drawRobot(pose, covariance)

	hold on;
	dim = 0.25;
	arr_len = 0.5;

	drawShape('rect', [pose(1), pose(2), dim, dim, pose(3)], 'fill','g');

end
