function done = drawLandmarks_slam(mean, state_to_id_map)
	
	color = 'r';
	mode = 'fill';


	N = size(mean,1)/2;
	radius = 0.1;
	for i=1:2:2*N

		land_id = state_to_id_map((i+1)/2); % recover the id of the landmark
		land_x = mean(i,1);
		land_y = mean(i+1,1);

		drawShape('circle', [land_x, land_y, radius], mode, color)
		hold on;
		drawLabels(land_x, land_y, land_id, '%d');
		hold on;
	end


end
