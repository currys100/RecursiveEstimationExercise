function [postParticles, redist_count] = Estimator(prevPostParticles, sens, act, init, N, K)
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
% if(nargin < 4) % TO DO change back to 4! 
%     % if not, set to default value:
%     init = 0;
% end

% if(nargin < 5) % TO DO change back to 4! 
%     % if not, set to default value:
%     init = 0;
% end

%% room params
% Room is size Lx2L, where L is a known constant set by KC.L

Ly = KC.L ;
Lx = 2*Ly ;

%% Mode 1: Initialization
% TO DO: how to tune the number of particles?
% Set number of particles:
% N = 100; % obviously, you will need more particles than 10.

if (init)
    % Do the initialization of your estimator here!
    % These particles are the posterior particles at discrete time k = 0
    % which will be fed into your estimator again at k = 1
    % Replace the following:
    %     postParticles.x = zeros(2,N);
    %     postParticles.y = zeros(2,N);
    %     postParticles.h = zeros(2,N);
    
    % randomly distribute the initial estimates between the 2 corners.
    % robot A is always on the west side of the room; robot B is always on
    % the left side.
    % postparticles is a 2xN matrix (2 robots x N samples).
    postParticles.x = [ones(1,N)*Lx ; zeros(1,N)] ;
    postParticles.y = [zeros(2,ceil(N/2)) ones(2,floor(N/2))*Ly ] ;
    % TO DO: make sure robots init headings point into the room
    %     postParticles.h = [pi/2+rand([2,ceil(N/2)])*pi/2 pi+rand([2,N/4])*pi/2 pi*3/2+rand([2,N/4])*pi/2 rand([2,N/4])*pi/2 ] ;
    postParticles.h = rand(2,N)*2*pi ;
    redist_count = NaN ; 
    % and leave the function
    return;
end % end init

%% Mode 2: Estimator iteration.
% If init = 0, we perform a regular update of the estimator.

% Implement your estimator here!

%% Step 1: Prior Update

% Generate PDF of Process Noise and get N samples:
lower = -KC.vsbar ;
peak = 0 ;
upper = KC.vsbar ;
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
x_p_update.x(2,:) = prevPostParticles.x(2,:) + velocity.B(1,:)*KC.ts ;
x_p_update.y(1,:) = prevPostParticles.y(1,:) + velocity.A(2,:)*KC.ts ;
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
    if hit.A.north(1,p)
        vj_error = heading_noise(prevPostParticles.h(1,p), 0) ;
        x_p_update.h(1,p) = (2*pi - prevPostParticles.h(1,p))*(1 + vj_error) ;
        x_p_update.y(1,p) = Ly - (x_p_update.y(1,p)-Ly) ; % Ly - error
    elseif hit.A.south(1,p)
        vj_error = heading_noise(prevPostParticles.h(1,p), 0) ;
        x_p_update.h(1,p) = (2*pi - prevPostParticles.h(1,p))*(1 + vj_error) ;
        x_p_update.y(1,p) = abs(x_p_update.y(1,p)) ;
    elseif hit.A.east(1,p)
        vj_error = heading_noise(prevPostParticles.h(1,p), pi/2) ;
        x_p_update.h(1,p) = (pi - prevPostParticles.h(1,p))*(1 + vj_error) ;
        x_p_update.x(1,p) = Lx - (x_p_update.x(1,p)-Lx) ; % Lx - error
    elseif hit.A.west(1,p)
        vj_error = heading_noise(prevPostParticles.h(1,p), pi/2) ;
        x_p_update.h(1,p) = (pi - prevPostParticles.h(1,p))*(1 + vj_error) ;
        x_p_update.x(1,p) = abs(x_p_update.x(1,p)) ;
    elseif hit.B.north(1,p)
        vj_error = heading_noise(prevPostParticles.h(2,p), 0) ;
        x_p_update.h(2,p) = (2*pi - prevPostParticles.h(2,p))*(1 + vj_error)  ;
        x_p_update.y(2,p) = Ly - (x_p_update.y(2,p)-Ly) ; % Ly - error
    elseif hit.B.south(1,p)
        vj_error = heading_noise(prevPostParticles.h(2,p), 0) ;
        x_p_update.h(2,p) = (2*pi - prevPostParticles.h(2,p))*(1 + vj_error)  ;
        x_p_update.y(2,p) = abs(x_p_update.y(2,p)) ;
    elseif hit.B.east(1,p)
        vj_error = heading_noise(prevPostParticles.h(2,p), pi/2) ;
        x_p_update.h(2,p) = (pi - prevPostParticles.h(2,p))*(1 + vj_error) ;
        x_p_update.x(2,p) = Lx - (x_p_update.x(2,p)-Lx) ; % Lx - error
    elseif hit.B.west(1,p)
        vj_error = heading_noise(prevPostParticles.h(2,p), pi/2) ;
        x_p_update.h(2,p) = (pi - prevPostParticles.h(2,p))*(1 + vj_error) ;
        x_p_update.x(2,p) = abs(x_p_update.x(2,p)) ;
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
% TO DO: make this a function. only execute if there are any sensor
% measurements.

% calculate distance from sensor to robot, based on x_p_update estimate.
% distance is a 2xN matrix containing distances to each sensor
distance.s1 = sqrt((Lx-x_p_update.x).^2 + x_p_update.y.^2 ) ;
distance.s2 = sqrt((Lx-x_p_update.x).^2 + (Ly-x_p_update.y).^2 ) ;
distance.s3 = sqrt(x_p_update.x.^2 + (Ly-x_p_update.y).^2 ) ;
distance.s4 = sqrt(x_p_update.x.^2 + x_p_update.y.^2 ) ;

% difference between z_meas and the distance (calculated from the prior) = z_noise.
% determine noise for each sensor to each robot just in case we detect the
% wrong robot.
z_noise.s1.A = abs(sens(1,1) - distance.s1(1,:)) ; % s1 to A
z_noise.s1.B = abs(sens(1,1) - distance.s1(2,:)) ; % s1 to B
z_noise.s2.A = abs(sens(2,1) - distance.s2(1,:)) ; % s2 to A
z_noise.s2.B = abs(sens(2,1) - distance.s2(2,:)) ; % s2 to B
z_noise.s3.A = abs(sens(3,1) - distance.s3(1,:)) ; % s3 to A
z_noise.s3.B = abs(sens(3,1) - distance.s3(2,:)) ; % s3 to B
z_noise.s4.A = abs(sens(4,1) - distance.s4(1,:)) ; % s4 to A
z_noise.s4.B = abs(sens(4,1) - distance.s4(2,:)) ; % s4 to B

% the probability of a sensor measurement with this noise is:
% sbar0 = p(right robot) ; sbar1 = p(wrong robot)
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
f_z_given_x.s1 = f_z_given_xs.s1.sbar0*(1-KC.sbar) + sum(f_z_given_xs.s1.sbar1)*KC.sbar ;
f_z_given_x.s2 = f_z_given_xs.s2.sbar0*(1-KC.sbar) + sum(f_z_given_xs.s2.sbar1)*KC.sbar ;
f_z_given_x.s3 = f_z_given_xs.s3.sbar0*(1-KC.sbar) + sum(f_z_given_xs.s3.sbar1)*KC.sbar ;
f_z_given_x.s4 = f_z_given_xs.s4.sbar0*(1-KC.sbar) + sum(f_z_given_xs.s4.sbar1)*KC.sbar ;

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
z_weights.A = z_weights.A / sum(z_weights.A) ;
z_weights.B = z_weights.B / sum(z_weights.B) ;


%% Resample: find the particle corresponding to the r bin

% in the case where all particles are outside the sensor range and
% therefore have zero (or equal) weight, we should resample from previous
% particle estimates equally.
postParticles.x = zeros(2,N);
postParticles.y = zeros(2,N);
postParticles.h = zeros(2,N);

% if neither test passes for any particle, then redistribute the particles.
tests_failed.A = 1;
tests_failed.B = 1;

% determine number of particles we're resampling from:
resample_set_count.A = sum(z_weights.A > 0) ;
resample_set_count.B = sum(z_weights.B > 0) ;

sums_z_weights.A = cumsum(z_weights.A) ; 
sums_z_weights.B = cumsum(z_weights.B) ; 

for j=1:N
    % choose a random r between 0 and 1
    r = rand() ;
    
    p_ind.A = find(sums_z_weights.A > r, 1) ; 
    p_ind.B = find(sums_z_weights.B > r, 1) ;
    
%     % determine which particle falls into the r bin by satisfying 2 tests
%     for p=1:N
%         
%         test1A = sum(z_weights.A(1:p)) >= r ;
%  
% %         test2A = sum(z_weights.A(1:p-1)) < r ;
%         test1B = sum(z_weights.B(1:p)) >= r ;
%         test2B = sum(z_weights.B(1:p-1)) < r ;
%         
        if ~isempty(p_ind.A)  
            postParticles.x(1,j) = x_p_update.x(1,p_ind.A)  ;
            postParticles.y(1,j) = x_p_update.y(1,p_ind.A)  ;
            postParticles.h(1,j) = x_p_update.h(1,p_ind.A)  ;
            tests_failed.A = 0 ;
        end % end if
%         
        if ~isempty(p_ind.B) 
            postParticles.x(2,j) = x_p_update.x(2,p_ind.B) ;
            postParticles.y(2,j) = x_p_update.y(2,p_ind.B) ;
            postParticles.h(2,j) = x_p_update.h(2,p_ind.B) ;
            tests_failed.B = 0 ;
        end % end if
%     end % end p=1:N
end % end for j=1:N

% K = 0.05 ; % K value if we do NOT redistribute particles

redist_count = 0 ; 

% if tests fail for all particles, redistribute particles
if tests_failed.A
    redist_particles = redistribute_particles('robotA', sens, N, Lx, Ly) ;
    postParticles.x(1,:) = redist_particles.x ;
    postParticles.y(1,:) = redist_particles.y ;
    postParticles.h(1,:) = rand(1, N)*2*pi ;
    redist_count = redist_count + 1 ; 
%     K = 3*K ; 
end
if tests_failed.B
    redist_particles = redistribute_particles('robotB', sens, N, Lx, Ly) ;
    postParticles.x(2,:) = redist_particles.x ;
    postParticles.y(2,:) = redist_particles.y ;
    postParticles.h(2,:) = rand(1, N)*2*pi ;
    redist_count = redist_count + 1 ; 
%     K = 3*K ; 
end

postParticles = roughen(postParticles, N, K, resample_set_count) ; 
 
end % end estimator


%% Nested functions

%% Roughening

% define signma_i = K*E*N^(-1/d) = standard dev of the noise to add to the
% particle.
function roughened_particles = roughen(particles, N, K, resample_size)
% K = 0.05 ; % K = tuning param << 1
% K = K*(1+ (N-resample_size.A)/N ) ; % scale K according to resample set
dimension = 3 ; % dimensions of state vector (x, y, h)

% roughen x,y coordinates: 
Ei.A.x = max(particles.x(1,:)) - min(particles.x(1,:)) ;
Ei.A.y = max(particles.y(1,:)) - min(particles.y(1,:)) ;
Ei_total.A = sqrt(Ei.A.x^2 + Ei.A.y^2) ;
% Ei_total.A = 1 ; 
sigma_i.A = K*Ei_total.A*N^(-1/dimension) ; % dimension = 2 for x,y coordinates
roughening_pdf.A = makedist('Normal', 'mu', 0, 'sigma' , sigma_i.A ) ;
roughening_noise.A = random(roughening_pdf.A, 1, N) ;
roughened_particles.x(1,:) = particles.x(1,:) + roughening_noise.A ;
roughened_particles.y(1,:) = particles.y(1,:) + roughening_noise.A ;
sprintf('sigmaAxy = %d', sigma_i.A) ;
% roughen heading: 
Ei.A.h = max(particles.h(1,:)) - min(particles.h(1,:)) ;
sigma_i.Ah = K*Ei.A.h*N^(-1/dimension) ; 
roughening_pdf.Ah = makedist('Normal', 'mu', 0, 'sigma' , sigma_i.Ah ) ;
roughening_noise.Ah = random(roughening_pdf.Ah, 1, N) ;
roughened_particles.h(1,:) = particles.h(1,:) + roughening_noise.Ah ;
sprintf('sigmaAh = %d', sigma_i.Ah) ;

if size(particles.x,1)==2
    K = K*(1+ (N-resample_size.B)/N ) ; % scale K according to resample set

    Ei.B.x = max(particles.x(2,:)) - min(particles.x(2,:)) ;
    Ei.B.y = max(particles.y(2,:)) - min(particles.y(2,:)) ;
    Ei.B.h = max(particles.h(2,:)) - min(particles.h(2,:)) ;
    Ei_total.B = sqrt(Ei.B.x^2 + Ei.B.y^2) ;
    sigma_i.B = K*Ei_total.B*N^(-1/dimension) ;
    roughening_pdf.B = makedist('Normal', 'sigma' , sigma_i.B ) ;
    roughening_noise.B = random(roughening_pdf.B, 1, N) ;
    roughened_particles.x(2,:) = particles.x(2,:) + roughening_noise.B ;
    roughened_particles.y(2,:) = particles.y(2,:) + roughening_noise.B ;
    sprintf('sigmaBxy = %d', sigma_i.B) ;
    
    % roughen heading:
    Ei.B.h = max(particles.h(2,:)) - min(particles.h(2,:)) ;
    sigma_i.Bh = K*Ei.B.h*N^(-1/dimension) ;
    sprintf('sigmaBh = %d', sigma_i.Bh)  ;
    roughening_pdf.Bh = makedist('Normal', 'mu', 0, 'sigma' , sigma_i.Bh ) ;
    roughening_noise.Bh = random(roughening_pdf.Bh, 1, N) ;
    roughened_particles.h(2,:) = particles.h(2,:) + roughening_noise.Bh ;
end % end if
end % end function roughen

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

end % end vj_error


function new_particles = redistribute_particles(robot, sens, N, Lx, Ly)
% if all particles have equal weights, then we have lost track of the
% robot and should randomly redistribute them.

if robot == 'robotA'
    if ~isinf(sens(1)) && ~isinf(sens(2))
        % redistribute sbar% of particles for robot A;
        new_particles.h = rand(1,N)*2*pi ;
        theta_s1_s2 =  [rand(1,floor(N/2))*pi/2 + pi/2 rand(1,floor(N/2))*pi/2 + pi ] ;
        new_particles.x = [Lx+sens(1)*cos(theta_s1_s2(1:floor(N/2))) Lx+sens(2)*cos(theta_s1_s2(floor(N/2+1):end)) ]; % cos(theta) < 0, so add this to Lx
        new_particles.y = [sens(1)*sin(theta_s1_s2(1:floor(N/2))) Ly+sens(2)*sin(theta_s1_s2(floor(N/2)+1:end))] ;
    elseif ~isinf(sens(1))
        new_particles.h = rand(1,N)*2*pi ;
        theta_s1 = rand(1,N)*pi/2 + pi/2 ;
        new_particles.x = Lx + sens(1)*cos(theta_s1) ; % cos(theta) < 0, so add this to Lx
        new_particles.y = sens(1)*sin(theta_s1) ;
    elseif ~isinf(sens(2))
        new_particles.h = rand(1,N)*2*pi ;
        theta_s2 = rand(1,N)*pi/2 + pi ;
        new_particles.x = Lx + sens(2)*cos(theta_s2) ;
        new_particles.y = Ly+sens(2)*sin(theta_s2) ;
    else
        new_particles.h = rand(1,N)*2*pi ;
        new_particles.x = rand(1,N)*Lx;
        new_particles.y = rand(1,N)*Ly ;
    end
end

if robot == 'robotB'
    if ~isinf(sens(3)) && ~isinf(sens(4))
        % redistribute sbar% of particles for robot A;
        new_particles.h = rand(1,N)*2*pi ;
        theta_s3_s4 =  [rand(1,floor(N/2))*pi/2 rand(1,floor(N/2))*pi/2 + 3*pi/2 ] ;
        new_particles.x = [sens(4)*cos(theta_s3_s4(1:floor(N/2))) sens(3)*cos(theta_s3_s4(floor(N/2+1):end)) ]; % cos(theta) < 0, so add this to Lx
        new_particles.y = [sens(4)*sin(theta_s3_s4(1:floor(N/2))) Ly+sens(3)*sin(theta_s3_s4(floor(N/2)+1:end))] ;
    elseif ~isinf(sens(4))
        new_particles.h = rand(1,N)*2*pi ;
        theta_s4 = rand(1,N)*pi/2 ;
        new_particles.x = sens(4)*cos(theta_s4) ; 
        new_particles.y = sens(4)*sin(theta_s4) ;
    elseif ~isinf(sens(3))
        new_particles.h = rand(1,N)*2*pi ;
        theta_s3 = rand(1,N)*pi/2 + 3*pi/2;
        new_particles.x = sens(3)*cos(theta_s3) ;
        new_particles.y = Ly+sens(3)*sin(theta_s3) ;
    else
        new_particles.h = rand(1,N)*2*pi ;
        new_particles.x = rand(1,N)*Lx;
        new_particles.y = rand(1,N)*Ly ;
    end
end


end



