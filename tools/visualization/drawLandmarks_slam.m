function done = drawLandmarks_slam(mean, state_to_id_map)
	
	color = 'r';
	mode = 'fill';


	N = size(mean,1)/3;
	radius = 0.1;
	#disp("N")
	#disp(N)
	for i=1:3:3*N
		#disp("i_LAND")
		#disp(i)
		land_id = state_to_id_map((i+2)/3); % recover the id of the landmark
		land_x = mean(i,1);
		land_y = mean(i+1,1);

		drawShape('circle', [land_x, land_y, radius], mode, color)
		hold on;
		drawLabels(land_x, land_y, land_id, '%d');
		hold on;
	end


end
