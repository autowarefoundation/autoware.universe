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
dt = 1/10;

%% Dimensions and Optimization Parameters
n = 4; % state dim with an additional disturbance states
m = 2; % control dim  - number of observable states
 
for i=1:7
    Xrr{i} = sdpvar(n); 
    Yrr{i} = sdpvar(m, n) ;
end

Xmin=sdpvar(n);

%% Quadratic cost 
Q=eye(n)*5; %diag([0.01,0.01,1,0.01]);%eye(n);
Q =sqrt(Q); 
 
R= eye(m)*5;
R = sqrt(R);

epsilon=0.001;
Q_eps=sqrtm(Q+epsilon);

%% Constraints
constraints = [];

%% Eigen value location constraints.
rho_r = 0.05;  % constraints the magnitude of imag value.
rho_p1 = 0.05; % positivity constraint, eigen values should be > 
rho_p2 = 0.9; % eigenvalues <
rho_s  = 0.2; % stability constraint z*z' = r < 1-rho_s 
 
[grow, gcol] = size(xugrid);

%% iterate over state and control grids stored by rows in xugrid 
for i=1:grow     
    
    % ey_grid, eyaw_grid, v_grid, delta_grid, kappa_grid, gdelta_input, a_grid, ddot_grid, wa_grid
 
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
    [t1, t2, t3, t4, t5, t6] = GetThetas_control(x, kappai, L);

    % Get Time Derivatives of Thetas
    [td1, td2, td3, td4, td5, td6] = GetThetasDot_control(x, u, kappai,tau_steering, L);   

    % Next thetas 
    tn1 = t1 + dt*td1;
    tn2 = t2 + dt*td2;
    tn3 = t3 + dt*td3;
    tn4 = t4 + dt*td4;
    tn5 = t5 + dt*td5;
    tn6 = t6 + dt*td6;

    Y  = Yrr{7}+t1*Yrr{1} + t2*Yrr{2} + t3*Yrr{3} + t4*Yrr{4} + t5*Yrr{5} + t6*Yrr{6};
    X  = Xrr{7}+t1*Xrr{1} + t2*Xrr{2} + t3*Xrr{3} + t4*Xrr{4}+ t5*Xrr{5} + t6*Xrr{6};
    Xn = Xrr{7}+tn1*Xrr{1} + tn2*Xrr{2} + tn3*Xrr{3} + tn4*Xrr{4}+ tn5*Xrr{5} + tn6*Xrr{6};

    constraints=[constraints; Xmin<=X];

    [A, B, C, D] = getAB_control_steering_delay(x,  kappai, tau_steering, L);

    A(3,3)=A(3,3) + A(3,3)* w2i; % tau uncertainty
    B(3,1)=B(3,1) + B(3,1)* w2i;

    sys_cont = ss(A, B, C, D);
    sys_disc = c2d(sys_cont, dt, 'tustin');  
    
    A = sys_disc.A;
    B = sys_disc.B;
 

    dotX =td1*Xrr{1} + td2*Xrr{2} + td3*Xrr{3} + td4*Xrr{4}+ td5*Xrr{5} + td6*Xrr{6};

    ineq=[X, X*A'+Y'*B', Q_eps*X, (R*Y)';...
    (X*A'+Y'*B')', Xn, zeros(n, n), zeros(n,m); ...
    (Q_eps*X)',zeros(n, n), eye(n), zeros(n, m); ...
    R*Y, zeros(m, n), zeros(m, n), eye(m)];

%     ineq=[X, A*X+B*Y;...
%     (A*X+B*Y)', Xn];

    % Huwitz stability   
    constraints=[constraints;ineq>=0];

    %{
    Pole clustering, L@P + M'@ (AP) + (PA)@M where (AP) is a complex variable. 
    %}

%     Z = X*A'+Y'*B'; 
  
    % stability radius (1-rho_s)
%     stab1_ineq = (1-rho_s)*X +  Z ;
%     stab2_ineq = (1-rho_s)*X'+  Z';
%     constraints = [constraints; stab1_ineq>=0;stab2_ineq>=0];
%  
%     % Real line, small imag
%     real_ima_ineq1 = 2*rho_r*X(1:3, 1:3)+Z - Z';
%     real_ima_ineq2 = 2*rho_r*X(1:3, 1:3)-Z + Z';
%     constraints = [constraints; real_ima_ineq1>=0;real_ima_ineq2>=0];
%  
    %Positivitiy constraints
%     positivity_ineq1 = 2*rho_p1*X + Z + Z';
%     constraints = [constraints; positivity_ineq1>=0];
 
end

%% Objective and problem setup
objective = -log(det(Xmin));

disp('Starting optimization');
options = sdpsettings('solver','mosek'); %sdpt3
solution = optimize(constraints, objective, options); 

%% Save results 

for i=1:7
    XXrrval{i}=value(Xrr{i}) ;   
    YYrrval{i}=value(Yrr{i}) ;
end

Xminval=value(Xmin);
inv((Xminval))

save('Lyaps_kinematic_control_steering_delay.mat','XXrrval','YYrrval', 'Xminval', ...
     'L','tau_steering', 'dt');


 

 

