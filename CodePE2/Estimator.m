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
% Room is size Lx2L

% let L1 = room.x and L2 = room.y
% could do symbolic, but we need actual numbers to feed back into the estimator.
% should distance be an absolute number, or some fraction of L1 and L2? 
% syms('L1', 'L2') ; 

% for now, assume room is 20x10 meters 
Ly = 10 ; 
Lx = 2*Ly ;

%% Mode 1: Initialization
% Set number of particles:
N = 10; % obviously, you will need more particles than 10.
if (init)
    % Do the initialization of your estimator here!
    % These particles are the posterior particles at discrete time k = 0
    % which will be fed into your estimator again at k = 1
    % Replace the following:
    postParticles.x = zeros(2,N);
    postParticles.y = zeros(2,N);
    postParticles.h = zeros(2,N);
    
    % randomly distribute the initial estimates. 
    % postparticles is a 2xN matrix (2 robots x N samples).
    postParticles.x = rand([2,N])*Lx; % rand[0,1]*room.x = random distribution across x dimension of room
    postParticles.y = rand([2,N])*Ly; % similar for y dimension of room
    postParticles.h = rand([2,N])*2*pi ; % random heading in interval [0, 2pi]
    
    % and leave the function
    return;
end % end init

%% Mode 2: Estimator iteration.
% If init = 0, we perform a regular update of the estimator.

% Implement your estimator here!

%% get sensor measurement
% S1 (lower right) and S2 (upper right) measure distances to robotA
% S3 (upper left) and S4 (lower left) measure distances to robotB

% determine distance to robotA: 
thetaA = arccos((sens(2)^2 - sens(1)^2 - Ly^2) / (-2*sens(1)*Ly)) ; % law of cosines

robotA.x = cos(pi/2 - thetaA) ;
robotA.y = sin(pi/2 - thetaA) ;

% determine distance to robotB: 
thetaB = arccos((sens(3)^2 - sens(4)^2 - Ly^2) / (2*sens(4)*Ly)) ; % law of cosines

robotB.x = cos(pi/2 - thetaB) ;
robotB.y = sin(pi/2 - thetaB) ;
%% resample. determine weights of particles 


%% propogate system model


% return postParticles
postParticles.x = zeros(2,N); % = x_{k-1} + vel_x * Ts (plus noise)
postParticles.y = zeros(2,N);
postParticles.h = zeros(2,N);




end % end estimator
