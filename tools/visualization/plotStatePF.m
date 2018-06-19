function out = plotStatePF(samples, weights, landmarks, best_particle, gt_pose)

hold off;

drawLandmarks(landmarks);
drawParticles(samples, weights, best_particle, gt_pose);

drawnow;

end

