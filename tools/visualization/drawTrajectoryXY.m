function drawTrajectoryXY(trajectory_)	

	#for each position in the trajectory
	for u = 1:rows(trajectory_)-1
	
	  #draw line from begin to end
	  x_begin = trajectory_(u, 1);
	  y_begin = trajectory_(u, 2);
	  x_end   = trajectory_(u+1, 1);
	  y_end   = trajectory_(u+1, 2);
    plot([x_begin x_end], [y_begin y_end], "b", "linewidth", 2);
    hold on;
	endfor
endfunction

