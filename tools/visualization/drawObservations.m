function drawObservations(map_, observations_, row_, col_)
  map_rows = rows(map_);
	hold on;
	
	#check the observation array
	if (observations_(1)) #UP
    rectangle("Position", [col_-1 map_rows-row_+1 1 0.5], "FaceColor", "blue", "EdgeColor", "none");
	endif
	if (observations_(2)) #DOWN
    rectangle("Position", [col_-1 map_rows-row_-0.5 1 0.5], "FaceColor", "blue", "EdgeColor", "none");
	endif
	if (observations_(3)) #LEFT
    rectangle("Position", [col_-1.5 map_rows-row_ 0.5 1], "FaceColor", "blue", "EdgeColor", "none");
	endif
	if (observations_(4)) #RIGHT
    rectangle("Position", [col_ map_rows-row_ 0.5 1], "FaceColor", "blue", "EdgeColor", "none");
	endif
	
	hold off;
endfunction

