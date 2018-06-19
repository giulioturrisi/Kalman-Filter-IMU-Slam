function drawBelief(state_belief_, map_)
	
	#invert belief value for plotting (0:black: 100% confidence, 1:white: 0% confidence)
	plotted_state_belief_ = flipud(ones(size(state_belief_)) - state_belief_);

  #plot a colormap with the respective belief values
	colormap(gray(64));
	hold on;
	image([0.5, columns(map_)-0.5], [0.5, rows(map_)-0.5], plotted_state_belief_*64);
	hold off;
endfunction

