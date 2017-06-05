function [postParticles] = Estimator(prevPostParticles, sens, act, init)
% [postParticles] = Estimator(prevPostParticles, sens, act, init)
%
% The estimator function. The function will be called in two different
% modes: If init==1, the estimator is initialized. If init == 0, the
% estimator does an iteration for a single sample time interval Ts (KC.ts)
% using the previous posterior particles passed to the estimator in
% prevPostParticles and the sensor measurements and control inputs.
%
% You must edit this function.
%
% Inputs:
%   prevPostParticles   previous posterior particles at discrete time k-1,
%                       which corresponds to continuous time t = (k-1)*Ts
%                       The variable is a struct whose fields are arrays
%                       that correspond to the posterior particle states.
%                       The fields are: (N is number of particles)
%                       .x = (2xN) array with the x-locations (metres)
%                       .y = (2xN) array with the y-locations (metres)
%                       .h = (2xN) array with the headings (radians)
%                       The first row in the arrays corresponds to robot A.
%                       The second row corresponds to robot B.
%
%   sens                Sensor measurements at discrete time k (t = k*Ts),
%                       [4x1]-array, an Inf entry indicates no measurement
%                       of the corresponding sensor.
%                       sens(1): distance reported by sensor 1 (metres)
%                       sens(2): distance reported by sensor 2 (metres)
%                       sens(3): distance reported by sensor 3 (metres)
%                       sens(4): distance reported by sensor 4 (metres)
%
%   act                 Control inputs u at discrete time k-1, which are
%                       constant during a time interval Ts:
%                       u(t) = u(k-1) for (k-1)*Ts <= t < k*Ts
%                       [2x1]-array:
%                       act(1): velocity of robot A, u_A(k-1) (metres/second)
%                       act(2): velocity of robot B, u_B(k-1) (metres/second)
%
%   init                Boolean variable indicating wheter the estimator
%                       should be initialized (init = 1) or if a regular
%                       estimator update should be performed (init = 0).
%                       OPTIONAL ARGUMENT. By default, init = 0.
%
% Outputs:
%   postParticles       Posterior particles at discrete time k, which
%                       corresponds to the continuous time t = k*Ts.
%                       The variable is a struct whose fields are arrays
%                       that correspond to the posterior particle states.
%                       The fields are: (N is number of particles)
%                       .x = (2xN) array with the x-locations (metres)
%                       .y = (2xN) array with the y-locations (metres)
%                       .h = (2xN) array with the headings (radians)
%                       The first row in the arrays corresponds to robot A.
%                       The second row corresponds to robot B.
%
% Class:
% Recursive Estimation
% Spring 2017
% Programming Exercise 2
%
% --
% ETH Zurich
% Institute for Dynamic Systems and Control
% Michael Muehlebach
% michaemu@ethz.ch

% Check if init argument was passed to estimator:
if(nargin < 4)
    % if not, set to default value:
    init = 0;
end

%% room params
% Room is size Lx2L, where L is a known constant set by KC.L

Ly = KC.L ;
Lx = 2*Ly ;

%% Mode 1: Initialization
% TO DO: how to tune the number of particles?
% Set number of particles:
N = 200; % obviously, you will need more particles than 10.
if (init)
    % Do the initialization of your estimator here!
    % These particles are the posterior particles at discrete time k = 0
    % which will be fed into your estimator again at k = 1
    % Replace the following:
    %     postParticles.x = zeros(2,N);
    %     postParticles.y = zeros(2,N);
    %     postParticles.h = zeros(2,N);
    
    % randomly distribute the initial estimates.
    % postparticles is a 2xN matrix (2 robots x N samples).
    
    postParticles.x = [ones(2,N/4)*Lx ones(2,N/4)*Lx zeros(2,N/4) zeros(2,N/4)] ;
    postParticles.y = [zeros(2,N/4) ones(2,N/4)*Ly ones(2,N/4)*Ly zeros(2,N/4)] ;
    % make sure robots init headings point into the room
    % TO DO: handle case where N/4 is not an integer.
    %     postParticles.h = [ pi/2+rand([2,N/4])*pi/2 pi+rand([2,N/4])*pi/2 pi*3/2+rand([2,N/4])*pi/2 rand([2,N/4])*pi/2 ] ;
    postParticles.h = rand(2,N)*2*pi ;
    
    % and leave the function
    return;
end % end init

%% Mode 2: Estimator iteration.
% If init = 0, we perform a regular update of the estimator.

% Implement your estimator here!

%% Step 1: Prior Update

% Generate PDF of Process Noise and get N samples:
vbar = KC.vbar;
lower = -vbar ;
peak = 0 ;
upper = vbar ;
% PDF of process noise
x_noise_pdf = makedist('Triangular','a',lower,'b',peak,'c',upper) ;
% N values from pdf for robots A and B:
x_noise_val.A = random(x_noise_pdf, 1, N) ;
x_noise_val.B = random(x_noise_pdf, 1, N) ;

% velocity.A, velocity.B are a 2xN matrices of x,y velocities for robots A and B
velocity.A = [act(1,1)*cos(prevPostParticles.h(1,:)) .* (1 + x_noise_val.A);
    act(1,1)*sin(prevPostParticles.h(1,:)) .* (1 + x_noise_val.A) ] ;
velocity.B = [act(2,1)*cos(prevPostParticles.h(2,:)) .* (1 + x_noise_val.B);
    act(2,1)*sin(prevPostParticles.h(2,:)) .* (1 + x_noise_val.B) ] ;

% prior updates for x,y position coordinates of robots A and B.
% KC.ts=timestep
x_p_update.x(1,:) = prevPostParticles.x(1,:) + velocity.A(1,:)*KC.ts ;
x_p_update.x(2,:) = prevPostParticles.x(2,:) + velocity.A(2,:)*KC.ts ;
x_p_update.y(1,:) = prevPostParticles.y(1,:) + velocity.B(1,:)*KC.ts ;
x_p_update.y(2,:) = prevPostParticles.y(2,:) + velocity.B(2,:)*KC.ts ;


%% prior update heading
% if we hit a wall, the new angle is  the ideal reflected angle + noise

% TO DO: how to make a pdf from a known distribution c*vj^2 ??
% making f_vj a uniform distribution for now.
% vj_pdf = makedist('Uniform', 'lower', -1, 'upper', 1) ;


% check if we've hit a wall:
hit.A.north = x_p_update.y(1,:) > Ly ;
hit.B.north = x_p_update.y(2,:) > Ly ;
hit.A.south = x_p_update.y(1,:) < 0 ;
hit.B.south = x_p_update.y(2,:) < 0 ;
hit.A.east = x_p_update.x(1,:) > Lx ;
hit.B.east = x_p_update.x(2,:) > Lx ;
hit.A.west = x_p_update.x(1,:) < 0 ;
hit.B.west = x_p_update.x(2,:) < 0 ;

% update heading with additional noise when we hit walls; else stay the
% same.
x_p_update.h = prevPostParticles.h ;
for p=1:N
    if hit.A.north(1,p) || hit.A.south(1,p)
        vj_error = heading_noise(prevPostParticles.h(1,p), 0) ;
        x_p_update.h(1,p) = (2*pi - prevPostParticles.h(1,p))*(1 + vj_error) ;
    elseif hit.A.east(1,p) || hit.A.west(1,p)
        vj_error = heading_noise(prevPostParticles.h(1,p), pi/2) ;
        x_p_update.h(1,p) = (pi - prevPostParticles.h(1,p))*(1 + vj_error) ;
    elseif hit.B.north(1,p) || hit.B.south(1,p)
        vj_error = heading_noise(prevPostParticles.h(2,p), 0) ;
        x_p_update.h(2,p) = (2*pi - prevPostParticles.h(2,p))*(1 + vj_error)  ;
    elseif hit.B.east(1,p) || hit.B.west(1,p)
        vj_error = heading_noise(prevPostParticles.h(2,p), pi/2) ;
        x_p_update.h(2,p) = (pi - prevPostParticles.h(2,p))*(1 + vj_error) ;
    end
end


%% Generate PDF of Sensor Noise and get N samples:

wbar = KC.wbar;
lower = -wbar ;
peak = 0 ;
upper = wbar ;
% PDF of process noise
z_noise_pdf = makedist('Triangular','a',lower,'b',peak,'c',upper) ;

%% determine weights of particles

% calculate distance from sensor to robot, based on x_p_update estimate.
% distance is a 2xN matrix containing distances to each sensor
distance.s1 = sqrt((Lx-x_p_update.x).^2 + x_p_update.y.^2 ) ;
distance.s2 = sqrt((Lx-x_p_update.x).^2 + (Ly-x_p_update.y).^2 ) ;
distance.s3 = sqrt(x_p_update.x.^2 + (Ly-x_p_update.y).^2 ) ;
distance.s4 = sqrt(x_p_update.x.^2 + x_p_update.y.^2 ) ;

% difference between z_meas and the distance (calculated from the prior) = z_noise.
% determine noise for each sensor to each robot just in case we detect the
% wrong robot.
%         z_noise.A(i,:) = abs(sens(1,1) - distance.s1(1,:)) ;
z_noise.s1.A = abs(sens(1,1) - distance.s1(1,:)) ; % s1 to A
z_noise.s1.B = abs(sens(1,1) - distance.s1(2,:)) ; % s1 to B
z_noise.s2.A = abs(sens(2,1) - distance.s2(1,:)) ; % s2 to A
z_noise.s2.B = abs(sens(2,1) - distance.s2(2,:)) ; % s2 to B
z_noise.s3.A = abs(sens(3,1) - distance.s3(1,:)) ; % s3 to A
z_noise.s3.B = abs(sens(3,1) - distance.s3(2,:)) ; % s3 to B
z_noise.s4.A = abs(sens(4,1) - distance.s4(1,:)) ; % s4 to A
z_noise.s4.B = abs(sens(4,1) - distance.s4(2,:)) ; % s4 to B

% the probability of a sensor measurement with this noise is:
% robot A to s1 and s2
f_z_given_xs.s1.sbar0 = pdf(z_noise_pdf, z_noise.s1.A ) ;
f_z_given_xs.s1.sbar1 = pdf(z_noise_pdf, z_noise.s1.B ) ;
f_z_given_xs.s2.sbar0 = pdf(z_noise_pdf, z_noise.s2.A ) ;
f_z_given_xs.s2.sbar1 = pdf(z_noise_pdf, z_noise.s2.B ) ;
% robot B to s3 and s4
f_z_given_xs.s3.sbar0 = pdf(z_noise_pdf, z_noise.s3.B ) ;
f_z_given_xs.s3.sbar1 = pdf(z_noise_pdf, z_noise.s3.A ) ;
f_z_given_xs.s4.sbar0 = pdf(z_noise_pdf, z_noise.s4.B ) ;
f_z_given_xs.s4.sbar1 = pdf(z_noise_pdf, z_noise.s4.A ) ;

% apply Total Probability Theorem to find f_z_given_x for each sensor
f_z_given_x.s1 = f_z_given_xs.s1.sbar0*KC.sbar + f_z_given_xs.s1.sbar1*(1-KC.sbar) ;
f_z_given_x.s2 = f_z_given_xs.s2.sbar0*KC.sbar + f_z_given_xs.s2.sbar1*(1-KC.sbar) ;
f_z_given_x.s3 = f_z_given_xs.s3.sbar0*KC.sbar + f_z_given_xs.s3.sbar1*(1-KC.sbar) ;
f_z_given_x.s4 = f_z_given_xs.s4.sbar0*KC.sbar + f_z_given_xs.s4.sbar1*(1-KC.sbar) ;

% Unnormalized likelihood
f_x_prior.A = pdf(x_noise_pdf, x_noise_val.A) ; % probability of x_noise
f_x_prior.B = pdf(x_noise_pdf, x_noise_val.B) ; % probability of x_noise

% if measurement = inf, there is no measurement for this iteration
% likelihood of the state of robot A = f(s1)*f(s2)
if ~isinf(sens(1)) && ~isinf(sens(2))
    z_weights.A = (f_x_prior.A .* f_z_given_x.s1) .* (f_x_prior.A .* f_z_given_x.s2) ;
elseif ~isinf(sens(1)) && isinf(sens(2))
    z_weights.A = (f_x_prior.A .* f_z_given_x.s1) ;
elseif isinf(sens(1)) && ~isinf(sens(2))
    z_weights.A = (f_x_prior.A .* f_z_given_x.s2) ;
elseif isinf(sens(1)) && isinf(sens(2))
    z_weights.A = ones([1,N]) ;
end


% likelihood of the state of robot B = f(s3)*f(s4)
% TO DO: come back to this!
if ~isinf(sens(3)) && ~isinf(sens(4))
    z_weights.B = (f_x_prior.B .* f_z_given_x.s3) .* (f_x_prior.B .* f_z_given_x.s4) ;
elseif ~isinf(sens(3)) && isinf(sens(4))
    z_weights.B = (f_x_prior.B .* f_z_given_x.s3) ;
elseif isinf(sens(3)) && ~isinf(sens(4))
    z_weights.B = (f_x_prior.B .* f_z_given_x.s4) ;
elseif isinf(sens(3)) && isinf(sens(4))
    z_weights.B = ones([1,N]) ;
end

% Normalized likelihood
sum(z_weights.A) ;
sum(z_weights.B) ;
z_weights.A = z_weights.A / sum(z_weights.A) ;
z_weights.B = z_weights.B / sum(z_weights.B) ;


%% Resample: find the particle corresponding to the r bin

postParticles.x = zeros(2,N);
postParticles.y = zeros(2,N);
postParticles.h = zeros(2,N);

for j=1:N
    % choose a random r between 0 and 1
    r = rand() ;
    
    % determine which particle falls into the r bin by satisfying 2 tests
    for p=1:N
        
        test1A = sum(z_weights.A(1:p)) >= r ;
        test2A = sum(z_weights.A(1:p-1)) < r ;
        test1B = sum(z_weights.B(1:p)) >= r ;
        test2B = sum(z_weights.B(1:p-1)) < r ;
        
        if test1A && test2A
            postParticles.x(1,j) = x_p_update.x(1,p)  ;
            postParticles.y(1,j) = x_p_update.y(1,p)  ;
            postParticles.h(1,j) = x_p_update.h(1,p)  ;
        end
        
        if test1B && test2B
            postParticles.x(2,j) = x_p_update.x(2,p) ;
            postParticles.y(2,j) = x_p_update.y(2,p) ;
            postParticles.h(2,j) = x_p_update.h(2,p) ;
        end
        
    end
    
    
    %% Roughening
    
    
    
    %% Nested functions
    
    function vj_error = heading_noise(incident_angle, wall_angle)
    % function determines the refelcted ideal angle and the error
    % calculate heading noise.
    c = 3/2*KC.vbar^3 ; % first find c by integrating over [-vbar, vbar]:
    max_f_vj = c*KC.vbar^2 ; % calculate max f(vj) = c*vbar^2
    f_vj = rand()*max_f_vj ; % choose a random f(vj)
    vj_pos = sqrt(f_vj*c) ; % find the (positive only!) corresponding value of vj
    vj_neg = -vj_pos ;
    vj_pos_neg = [vj_pos vj_neg ] ;
    
    assign_pos_or_neg = randi(2) ;  % make vj randomly positive or negative
    vj = vj_pos_neg(assign_pos_or_neg) ;
    
    % find angle alpha (which is the angle between the wall and the incident
    % angle). heading error = alpha*vj
    alpha = incident_angle ;
    while (alpha - wall_angle) > pi/2
        alpha = alpha - pi/2 ;
    end
    
    vj_error = alpha*vj ;
    
    %     end % end function vj_error % TO DO : update matlab, uncomment this
    %     end statement
    
    
end % end estimator
    
