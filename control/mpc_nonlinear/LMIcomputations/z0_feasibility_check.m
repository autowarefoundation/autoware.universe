clc
clear

%% CHECK OBSERVER
load xugrid_check.mat % larger grid 
%%
 
disp('Verifying steering delay system observers ... ')
load Lyaps_steering_delay_observer.mat
[unstable, stable] = check_stability_steering_delay_observer(xugrid, XXrrval, YYrrval, tau_steering, L, dt);

  
fprintf('number of stable %i\n', stable)
fprintf('number of unstable %i\n', unstable)


%% CHECK CONTROLLER
clc 
clear
load xugrid_check.mat % larger grid 
% load xugrid_rate.mat % larger grid 
disp('Verifying steering delay system controllers ... ')

load Lyaps_kinematic_control_steering_delay.mat
[unstable, stable] = check_stability_control_steering_delay(xugrid, XXrrval, YYrrval, tau_steering, L, dt) ;

fprintf('number of stable %i\n', stable)
fprintf('number of unstable %i\n', unstable)