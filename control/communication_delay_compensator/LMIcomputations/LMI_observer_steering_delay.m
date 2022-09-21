%% Offline LMIs to compute P,K - continous-time formulation
clear all;
clc;

%% Load files
load xugrid_rate

%%
% 
% [ey, epsi, v, delta] error model 
% In goodnotes, SCVx implementation LPV folder. 

%% Delay Model
tau_steering = 0.24 ;
L = 2.75 ;
dt = 1/30;

%% Dimensions and Optimization Parameters
n = 4; % state dim with an additional disturbance states
m = 3; % control dim  - number of observable states

rho_c = 0.1;  % decay rate

% Quadratic cost 
Q=eye(n)*.01; %diag([0.01,0.01,1,0.01]);%eye(n);
Q =sqrt(Q); 
 
R=eye(m)*0.2; %0.5
R = sqrt(R);

epsilon=0.001;
Q_eps=sqrtm(Q+epsilon*0);

R_eps=sqrt(R);

 
for i=1:5
    Xrr{i} = sdpvar(n); 
    Yrr{i} = sdpvar(m, n) ;
end

Xmin=sdpvar(n);

% Constraints
constraints = [];

[grow, gcol] = size(xugrid);

%% iterate over state and control grids stored by rows in xugrid 
for i=1:grow     
    
    % ey_grid, eyaw_grid, v_grid, delta_grid, kappa_grid, gdelta_input, a_grid, ddot_grid, wa_grid
    % 
    
    ey = xugrid(i, 1);      % lateral error
    epsi = xugrid(i, 2);    % heading error
    vi = xugrid(i, 3);      % extract speed
    deltai = xugrid(i, 4);  % extract steering
    kappai = xugrid(i, 5);  % curvature grid 
    usteeri = xugrid(i, 6);  % curvature grid 

    uai = xugrid(i, 7); % extract acceleration input
    udi = xugrid(i, 8); % extract delta input
    w2i = xugrid(i, 9); % extract input uncertainty for acc [-0.1, 0.1]
    
    % Collect x, u
    x =[ey;epsi;vi;deltai]; 
    u =[usteeri; uai;udi];
    
   %calculate Y and X for every point depending on the parameters at that point
   [t1, t2, t3, t4] = GetThetas_observer(x, kappai, L);
   
   % Get Time Derivatives of Thetas
   [td1, td2, td3, td4] = GetThetasDot_observer(x, u, kappai,tau_steering, L);   
   
   % Next thetas 
   tn1 = t1 + dt*td1;
   tn2 = t2 + dt*td2;
   tn3 = t3 + dt*td3;
   tn4 = t4 + dt*td4;
   
   
   Y  = Yrr{5}+t1*Yrr{1} + t2*Yrr{2} + t3*Yrr{3} + t4*Yrr{4};
   X  = Xrr{5}+t1*Xrr{1} + t2*Xrr{2} + t3*Xrr{3} + t4*Xrr{4};
   Xn = Xrr{5}+tn1*Xrr{1} + tn2*Xrr{2} + tn3*Xrr{3} + tn4*Xrr{4};
   
   constraints=[constraints; Xmin<=X];
   
  
   [A, B, C, D] = getAB_steering_delay_observer(x,  kappai, tau_steering, L);
   
   A(3,3)=A(3,3) + A(3,3)* w2i; % tau uncertainty
   %B(3,1)=B(3,1) + B(3,1)* w2i;
   
   sys_cont = ss(A, B, C, D);
   sys_disc = c2d(sys_cont, dt, 'tustin');  
   
   A = sys_disc.A';
   B = (sys_disc.C*sys_disc.A)'; 
   
   
   dotX =td1*Xrr{1} + td2*Xrr{2} + td3*Xrr{3} + td4*Xrr{4};
   
   ineq=[X, X*A'+Y'*B', Q_eps*X, (R*Y)';...
        (X*A'+Y'*B')', Xn, zeros(n, n), zeros(n,m); ...
        (Q_eps*X)',zeros(n, n), eye(n), zeros(n, m); ...
        R*Y, zeros(m, n), zeros(m, n), eye(m)];
        
   constraints=[constraints;ineq>=0];
 
end

%% Objective and problem setup
objective = -log(det(Xmin));

disp('Starting optimization');
options = sdpsettings('solver','mosek'); %sdpt3
solution = optimize(constraints, objective, options); 

%% Save results 

for i=1:5
    XXrrval{i}=value(Xrr{i}) ;   
    YYrrval{i}=value(Yrr{i}) ;
end

Xminval=value(Xmin);
inv((Xminval))

save('Lyaps_steering_delay_observer.mat','XXrrval','YYrrval', 'Xminval', ...
     'L','tau_steering', 'dt');


 

 

