% this function computes the transition of the robot after incorportating
% a movement u = [ux, uy, u_theta]
% in case of a differential drive robot, ignore uy (the case seen in the classroom)
% inputs:
%   u(1): offset on x
%   u(2): offset on y // ignore in case of differential drive
%   u(3): offset on theta

%   mu(1): x coord of robot w.r.t world
%   mu(2): y coord of robot w.r.t world
%   mu(3): angle of robot w.r.t world

% outputs:
%   mu_prime(1): x coord of robot w.r.t world, after transition
%   mu_prime(2): y coord of robot w.r.t world, after transition
%   mu_prime(3): angle of robot w.r.t world, after transition

function mu_prime = transition_model(mu, u)

        mu_prime = mu;
        mu_x = mu(1);
        mu_y = mu(2);   
        ux = u(1);
        utheta = u(3);
        mu_theta = mu(3);
        c = cos(mu_theta);
        s = sin(mu_theta);

        %       (x' ) = (    x + ux*cos(th)     )
        %       (y' ) = (    y + ux*sin(th)     )
        %       (th') = (    th + uth           )

        mu_prime(1) = mu_x + ux*c;
        mu_prime(2) = mu_y + ux*s;
        mu_prime(3) = mu_theta + utheta;


end
