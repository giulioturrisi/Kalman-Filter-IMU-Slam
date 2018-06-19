function out = drawParticles(samples, weights, best_particle, gt_pose)

	dim_particles = size(samples,2);

	hold on;
	plot(samples(1,:), samples(2,:), 'b.');

	hold on;
	plot(gt_pose.x, gt_pose.y, 'c.');
	hold on;
	drawCircle(gt_pose.x, gt_pose.y, 0.5);

	if best_particle > 0
		hold on;
		drawCircle(samples(1,best_particle), samples(2,best_particle), 0.5);
		hold on;
		drawRobot(samples(:,best_particle));
	endif

endfunction
