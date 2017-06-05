simConst = SimulationConstants();
estConst = EstimatorConstants();
doplot = false;
seed = 0;

%% Q_v = 0.01
estConst.VelocityInputPSD = 0.01;
tracked_error_1 = zeros(50,1);
for i=1:50
    tracked_error_1(i,1) = run(simConst,estConst,doplot,seed);
end
mean_1 = mean(tracked_error_1)
var_1 = var(tracked_error_1)

figure(1)
histogram(tracked_error_1,linspace(0,3,31))
title(['Q_v = 0.01, mean = ' num2str(mean_1) ' varaince = ' num2str(var_1)])
%% Q_v = 0.1
estConst.VelocityInputPSD = 0.1;
tracked_error_2 = zeros(50,1);
for i=1:50
    tracked_error_2(i,1) = run(simConst,estConst,doplot,seed);
end
mean_2 = mean(tracked_error_2)
var_2 = var(tracked_error_2)

figure(2)
histogram(tracked_error_2,linspace(0,3,31))
title(['Q_v = 0.1, mean = ' num2str(mean_2) ' varaince = ' num2str(var_2)])
%% Q_v = 1
estConst.VelocityInputPSD = 1;
tracked_error_3 = zeros(50,1);
for i=1:50
    tracked_error_3(i,1) = run(simConst,estConst,doplot,seed);
end
mean_3 = mean(tracked_error_3)
var_3 = var(tracked_error_3)

figure(3)
histogram(tracked_error_3,linspace(0,3,31))
title(['Q_v = 1, mean = ' num2str(mean_3) ' varaince = ' num2str(var_3)])

fprintf('The best noise variance is related to the CompassNoise, the lowest\n variance is obtained for Q_v = sig_c\n')
