
function [td1, td2, td3, td4, td5, td6]=GetThetasDot_control(x,u, kappai, tau, L) 
    
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
    t2 = sin(eyaw);
    t3 = -kp1_sqr * t1 ;
    t4 = kp1*v*sin(eyaw);    
    t5 = tan(delta)/L - kp1 * cos(eyaw);
    t6 = v*(tan(delta)^2 + 1)/L;
    
    td1 =  ua* cos(eyaw) - v*sin(eyaw)* eydot;    
    td2 =  cos(eyaw)*eyawdot;   
    
    td3 =  -kp1_sqr * (2*kp1_sqr)*v*cos(eyaw)*eydot - ua*kp1_sqr*cos(eyaw) + kp1_sqr*v*sin(eyaw)*eyawdot ;     
    td4 =  kp1_sqr * eydot*v*sin(eyaw) + kp1_sqr *(ua*sin(eyaw) + v*cos(eyaw)*eyawdot);
    
    
    td5 =  delta_dot * (1 + tan(delta)^2)/L + kp1*sin(eyaw)*eyawdot -kp1_sqr*cos(eyaw)*eydot ;   
    td6 =  ua*(1 +tan(delta)^2) + 2*tan(delta)*v*(tan(delta)^2 + 1)*delta_dot / L; 

end 