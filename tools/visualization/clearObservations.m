function clearObservations(observations_, row_, col_, map_)
	map_rows = rows(map_);
	hold on;

	#check the observation array
	if (observations_(1)) #UP
	  if (map_(row_-1, col_)) 
	    rectangle("Position", [col_-1 map_rows-row_+1 1 1], "FaceColor", "black");
	  else
      rectangle("Position", [col_-1 map_rows-row_+1 1 1], "FaceColor", "white");
    endif
	endif
	if (observations_(2)) #DOWN
	  if (map_(row_+1, col_))
	    rectangle("Position", [col_-1 map_rows-row_-1 1 1], "FaceColor", "black");
	  else
      rectangle("Position", [col_-1 map_rows-row_-1 1 1], "FaceColor", "white");
    endif
	endif
	if (observations_(3)) #LEFT
	  if (map_(row_, col_-1))
	    rectangle("Position", [col_-2 map_rows-row_ 1 1], "FaceColor", "black");
	  else
      rectangle("Position", [col_-2 map_rows-row_ 1 1], "FaceColor", "white");
    endif
	endif
	if (observations_(4)) #RIGHT
	  if (map_(row_, col_+1))
	    rectangle("Position", [col_ map_rows-row_ 1 1], "FaceColor", "black");
	  else
      rectangle("Position", [col_ map_rows-row_ 1 1], "FaceColor", "white");
    endif
	endif
	
	hold off;
endfunction

