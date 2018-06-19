function drawMap(map_)
	hold on;
	map_rows = rows(map_);
	map_cols = columns(map_);
	for row = 1:map_rows
		for col = 1:map_cols
		
		  #draw a black block if the map element is occupied (e.g. wall)
		  #since the plotting command does not reason in matrix rows we have to shift them
			if(map_(row, col) == 1)
				rectangle("Position", [col-1 map_rows-row 1 1], "FaceColor", "black");
			else
				rectangle("Position", [col-1 map_rows-row 1 1], "FaceColor", "white");
			endif
		endfor
	endfor
	hold off;
endfunction

