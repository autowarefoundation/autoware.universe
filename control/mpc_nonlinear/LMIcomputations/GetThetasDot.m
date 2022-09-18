
function [td1, td2, td3, td4]=GetThetasDot(x,u, kappai, tau, L) 
    
    % Extract states and control
    ey = x(1);
    eyaw = x(2);
    v = x(3);
    delta=x(4);
    
    u_steer_input = u(1);   % steering input
    ua = u(2);              % acceleration
    uddot = u(3);           % steering rate
    
    % Implicits 
    kp1 = kappai/(1-kappai*ey);
    kp1_sqr = kp1^2 ; 
    
    % Dynamics
    eydot = v*sin(eyaw); 
    eyawdot = v*tan(delta)/L - kp1*v*cos(eyaw);    
    delta_dot = -delta/tau + u_steer_input/tau;  
    
    
    t1 = v*cos(eyaw);    
    t2 = -kp1_sqr* t1;
    t3 =  kp1* v*sin(eyaw);
    t4 =  v/(L*cos(delta)^2);
    
    td1 =  ua* cos(eyaw) - v*sin(eyaw)* eydot;
    td2 =  -kp1_sqr * (2*kappai)*v*cos(eyaw) - kp1_sqr* td1;
    td3 = kp1_sqr * eydot*v*sin(eyaw) + ua*sin(eyaw) + v*cos(eyaw)*eyawdot;
    td4 =  ua/(L*cos(delta)^2) + 2*v*sin(delta)*delta_dot/(L*cos(delta)^3); 

end 