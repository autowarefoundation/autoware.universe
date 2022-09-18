
function [t1, t2, t3, t4]=GetThetas(x,kappai, L)
    % Extract states and control
    % states :  x =[ey;epsi;vi;deltai]; 
    ey = x(1);
    eyaw = x(2);
    v = x(3);
    delta=x(4);
    
    kp1 = kappai/(1-kappai*ey);
    kp1_sqr = kp1^2 ; 
    
    t1 = v*cos(eyaw);
    t2 = -kp1_sqr * t1 ;
    t3 =  kp1*v*sin(eyaw);    
    t4 = v/(L*cos(delta)^2);
 
end 