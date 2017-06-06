function [posEst,oriEst,driftEst, posVar,oriVar,driftVar,estState] = EstimatorV2(estState,actuate,sense,tm,estConst)
% [posEst,oriEst,driftEst, posVar,oriVar,driftVar,estState] =
%   Estimator(estState,actuate,sense,tm,estConst)
%
% The estimator.
%
% The function will be called in two different modes:
% If tm==0, the estimator is initialized; otherwise the estimator does an
% iteration step (compute estimates for the time step k).
%
% Inputs:
%   estState        previous estimator state (time step k-1)
%                   May be defined by the user (for example as a struct).
%   actuate         control input u(k), [1x2]-vector
%                   actuate(1): u_v, drive wheel angular velocity
%                   actuate(2): u_r, drive wheel angle
%   sense           sensor measurements z(k), [1x3]-vector, INF if no
%                   measurement
%                   sense(1): z_d, distance measurement
%                   sense(2): z_c, compass measurement
%                   sense(3): z_g, gyro measurement
%   tm              time, scalar
%                   If tm==0 initialization, otherwise estimator
%                   iteration step.
%   estConst        estimator constants (as in EstimatorConstants.m)
%
% Outputs:
%   posEst          position estimate (time step k), [1x2]-vector
%                   posEst(1): x position estimate
%                   posEst(2): y position estimate
%   oriEst          orientation estimate (time step k), scalar
%   driftEst        estimate of the gyro drift b (time step k), scalar
%   posVar          variance of position estimate (time step k), [1x2]-vector
%                   posVar(1): x position variance
%                   posVar(2): y position variance
%   oriVar          variance of orientation estimate (time step k), scalar
%   driftVar        variance of gyro drift estimate (time step k), scalar
%   estState        current estimator state (time step k)
%                   Will be input to this function at the next call.
%
%
% Class:
% Recursive Estimation
% Spring 2017
% Programming Exercise 1
%
% --
% ETH Zurich
% Institute for Dynamic Systems and Control
% Raffaello D'Andrea, Michael Muehlebach, Lukas Hewing
% michaemu@ethz.ch
% lhewing@ethz.ch
%
% --
% Revision history
% [19.04.11, ST]    first version by Sebastian Trimpe
% [30.04.12, PR]    adapted version for spring 2012, added unknown wheel
%                   radius
% [06.05.13, MH]    2013 version
% [23.04.15, MM]    2015 version
% [14.04.16, MM]    2016 version
% [05.05.17, LH]    2017 version


%% Mode 1: Initialization
if (tm == 0)
    %     Do the initialization of your estimator here!
    %     Replace the following:
    posEst = [estConst.TranslationStartBound*rand(),estConst.TranslationStartBound*rand()];
    oriEst = estConst.RotationStartBound*rand();
    driftEst = 0;
    posVar = [(estConst.TranslationStartBound^2)/3 (estConst.TranslationStartBound^2)/3];
    oriVar = (estConst.RotationStartBound^2)/3;
    driftVar = 0;%Q_b;
    estState.posEst = posEst;
    estState.oriEst = oriEst;
    estState.driftEst = driftEst;
    estState.posVar = posVar;
    estState.oriVar = oriVar;
    estState.driftVar = driftVar;
    estState.P = diag([oriVar,posVar,driftVar],0);
    return;
end


%% Mode 2: Estimator iteration.
% If we get this far tm is not equal to zero, and we are no longer
% initializing.  Run the estimator.
W = estConst.WheelRadius;
B = estConst.WheelBase;
u_v = actuate(1);
u_r = actuate(2);
Q_v = estConst.VelocityInputPSD;
Q_b = estConst.GyroDriftPSD;
sig_r = estConst.CompassNoise;
sig_g = estConst.GyroNoise;
sig_d = estConst.DistNoise;
z_c = sense(2);
z_g = sense(3);
z_d = sense(1);
% Step1
S_t = W*u_v*cos(u_r);

    function A = A_mat(ori_Est)
        A = [0,                   0, 0, 0;...
            -S_t*sin(ori_Est),    0, 0, 0;...
            S_t*cos(ori_Est),     0, 0, 0;...
            0,                    0, 0, 0];
    end

    function L = L_mat(ori_Est)
        L = ...
            [-(W/B)*u_v*sin(u_r) 0;...
            S_t*cos(ori_Est),                                          0;...
            S_t*sin(ori_Est),                                          0;...
            0,                                                         1];
    end


    function P_ode = odefunc(t,P_)
        P = convertVecToMatrix(P_(5:20,1));
        P_ode(1,1) = -(W/B)*u_v*sin(u_r);
        P_ode(2,1) = S_t*cos(P_(1,1));
        P_ode(3,1) = S_t*sin(P_(1,1));
        P_ode(4,1) = 0;
        A = A_mat(P_(1,1));
        L = L_mat(P_(1,1));
        P_ode(5:20,1) = convertMatrixToVec(A*P + P*A' +L*[Q_v, 0;0, Q_b]*L');
    end

% Solving the ode
tspan = [0, 0.1];
P_0 = convertMatrixToVec(estState.P);
x_0 = [estState.oriEst,estState.posEst,estState.driftEst]';
P_p_vec = ode45(@(t,P) odefunc(t,P), tspan, [x_0;P_0]);
P_p = convertVecToMatrix(P_p_vec.y(5:20,end));
x_p = P_p_vec.y(1:4,end);


estState.oriEst = x_p(1,1);
estState.posEst = [x_p(2,1), x_p(3,1)];
estState.driftEst = x_p(4,1);

% Step 2
% if (isinf(z_c))&&(isinf(z_g))&&(isinf(z_d))
%     x_m = x_p;
%     P_m = P_p;
% end
% if (isinf(z_c)==0)&&(isinf(z_g)==0)&&(isinf(z_d))
%     H = [1, 0, 0, 0; 1, 0, 0, 1];
%     M = eye(2);
%     R = diag([sig_r,sig_g],0);
%     K_k = P_p*H'/(H*P_p*H' + M*R*M');
%     x_m = x_p + K_k*([z_c,z_g]' - [estState.oriEst; estState.oriEst + estState.driftEst]);
%     P_m = (eye(4) - K_k*H)*P_p;
% end
% if (isinf(z_c)==0)&&(isinf(z_g))&&(isinf(z_d)==0)
%     H = [1, 0, 0, 0; 0, estState.posEst(1)/(sqrt(estState.posEst*estState.posEst')), estState.posEst(2)/(sqrt(estState.posEst*estState.posEst')), 0];
%     M = eye(2);
%     R = diag([sig_r,sig_d],0);
%     K_k = P_p*H'/(H*P_p*H' + M*R*M');
%     x_m = x_p + K_k*([z_c,z_d]' - [estState.oriEst; sqrt(estState.posEst*estState.posEst')]);
%     P_m = (eye(4) - K_k*H)*P_p;
% end
% if (isinf(z_c))&&(isinf(z_g)==0)&&(isinf(z_d)==0)
%     H = [ 1, 0, 0, 1; 0, estState.posEst(1)/(sqrt(estState.posEst*estState.posEst')), estState.posEst(2)/(sqrt(estState.posEst*estState.posEst')), 0];
%     M = eye(2);
%     R = diag([sig_g,sig_d],0);
%     K_k = P_p*H'/(H*P_p*H' + M*R*M');
%     vect = ([z_g,z_d]' - [estState.oriEst + estState.driftEst; sqrt(estState.posEst*estState.posEst')]);
%     x_m = x_p + K_k*vect;
%     P_m = (eye(4) - K_k*H)*P_p;
% end
% if (isinf(z_c)==0)&&(isinf(z_g))&&(isinf(z_d))
%     H = [1, 0, 0, 0];
%     M = 1;
%     R = sig_r;
%     K_k = P_p*H'/(H*P_p*H' + M*R*M');
%     vect = (z_c - estState.oriEst);
%     x_m = x_p + vect*K_k;
%     P_m = (eye(4) - K_k*H)*P_p;
% end
% if (isinf(z_c))&&(isinf(z_g)==0)&&(isinf(z_d))
%     H = [1, 0, 0, 1];
%     M = 1;
%     R = sig_g;
%     K_k = P_p*H'/(H*P_p*H' + M*R*M');
%     vect = (z_g - estState.oriEst - estState.driftEst);
%     x_m = x_p + vect*K_k;
%     P_m = (eye(4) - K_k*H)*P_p;
% end
% if (isinf(z_c))&&(isinf(z_g))&&(isinf(z_d)==0)
%     H = [0, estState.posEst(1)/(sqrt(estState.posEst*estState.posEst')), estState.posEst(2)/(sqrt(estState.posEst*estState.posEst')), 0];
%     M = 1;
%     R = sig_d;
%     K_k = P_p*H'/(H*P_p*H' + M*R*M');
%     vect = (z_d - sqrt(estState.posEst*estState.posEst'));
%     x_m = x_p + vect*K_k;
%     P_m = (eye(4) - K_k*H)*P_p;
% end
% if (isinf(z_c)==0)&&(isinf(z_g)==0)&&(isinf(z_d)==0)
%     H = [1, 0, 0, 0; 1, 0, 0, 1; 0, estState.posEst(1)/(sqrt(estState.posEst*estState.posEst')), estState.posEst(2)/(sqrt(estState.posEst*estState.posEst')), 0];
%     M = eye(3);
%     R = diag([sig_r,sig_g,sig_d],0);
%     K_k = P_p*H'/(H*P_p*H' + M*R*M');
%     x_m = x_p + K_k*(sense' - [estState.oriEst; estState.oriEst + estState.driftEst; sqrt(estState.posEst*estState.posEst')]);
%     P_m = (eye(4) - K_k*H)*P_p;
% end

% H = [1, 0, 0, 0; 1, 0, 0, 1; 0, estState.posEst(1)/(sqrt(estState.posEst*estState.posEst')), estState.posEst(2)/(sqrt(estState.posEst*estState.posEst')), 0];
% M = eye(3);
% R = diag([sig_r,sig_g,sig_d],0);
% K_k = P_p*H'/(H*P_p*H' + M*R*M');
% z_c2 = z_c;
% if isinf(z_c2)
%     H(1,:) = zeros(1,4);
%     z_c2 = estState.oriEst;
% end
% z_g2 = z_g;
% if isinf(z_g2)
%     H(2,:) = zeros(1,4);
%     z_g2 = estState.oriEst + estState.driftEst;
% end
% z_d2 = z_d;
% if isinf(z_d2)
%     H(3,:) = zeros(1,4);
%     z_d2 = sqrt(estState.posEst*estState.posEst');
% end
%
% z = [z_c2; z_g2; z_d2];
% x_m_ = x_p + K_k*(z - [estState.oriEst; estState.oriEst + estState.driftEst; sqrt(estState.posEst*estState.posEst')]);
% P_m_ = (eye(4) - K_k*H)*P_p;

% Compass
[x_c, P_c] = compass(x_p, P_p, z_c);

% Gyro
[x_g, P_g] = gyro(x_c, P_c, z_g);

% Distance
[x_m, P_m] = distance(x_g, P_g, z_d);

% Replace the following:
estState.posEst = [x_m(2,1),x_m(3,1)];
estState.oriEst = x_m(1,1);
estState.driftEst = x_m(4,1);
estState.posVar = [P_m(2,2), P_m(3,3)];
estState.oriVar = P_m(1,1);
estState.driftVar = P_m(4,4);
estState.P = P_m;
posEst = estState.posEst;
oriEst = estState.oriEst;
driftEst = estState.driftEst;
posVar = estState.posVar;
oriVar = estState.oriVar;
driftVar = estState.driftVar;


    function [x_m1, P_m1] = compass(x_p1, P_p1, z1)
        if ~isinf(z1)
            H1 = [1,0,0,0];
            M1 = 1;
            R1 = [sig_r];
            K1 = P_p1*H1'/(H1*P_p1*H1' + M1*R1*M1');
            
            x_m1 = x_p1 + K1*(z1 - [x_p1(1)]);
            P_m1 = (eye(4) - K1*H1)*P_p1;
        else
            x_m1 = x_p1;
            P_m1 = P_p1;
        end
    end

    function [x_m1, P_m1] = gyro(x_p1, P_p1, z1)
        if ~isinf(z1)
            H2 = [1,0,0,1];
            M2 = 1;
            R2 = [sig_g];
            K2 = P_p1*H2'/(H2*P_p1*H2' + M2*R2*M2');
            
            x_m1 = x_p1 + K2*(z1 - (x_p1(1) + x_p1(4)));
            P_m1 = (eye(4) - K2*H2)*P_p1;
        else
            x_m1 = x_p1;
            P_m1 = P_p1;
        end
    end

    function [x_m1, P_m1] = distance(x_p1, P_p1, z1)
        if ~isinf(z1)
            H3 = [ 0, x_p1(2)/(sqrt(x_p1(2:3)'*x_p1(2:3))), x_p1(3)/(sqrt(x_p1(2:3)'*x_p1(2:3))), 0];
            M3 = 1;
            R3 = [sig_d];
            K3 = P_p1*H3'/(H3*P_p1*H3' + M3*R3*M3');
            
            x_m1 = x_p1 + K3*(z1 - sqrt(x_p1(2:3)'*x_p1(2:3)));
            P_m1 = (eye(4) - K3*H3)*P_p1;
        else
            x_m1 = x_p1;
            P_m1 = P_p1;
        end
    end


end
