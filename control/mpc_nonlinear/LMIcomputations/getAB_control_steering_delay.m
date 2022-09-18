function [A, B, C, D] = getAB_control_steering_delay(x,kappai, tau, L)
    n=4;
    m=2;
 
    
    % extract states and control
    ey = x(1);
    epsi = x(2);
    v = x(3);
    delta=x(4); 

    % compute lineariztion
    A=zeros(n);
    B=zeros(n,m);
    
    % states
    
    kp1 = kappai/(1-kappai*ey);
   
    
    A(1, 2) = v*cos(epsi);
    A(1, 3) = 0;    
    
    A(2, 1) = -kp1^2*A(1, 2);
    A(2, 2) = kp1*v*sin(epsi);
    A(2, 3) = v/(L*cos(delta)^2);    
    A(3, 3) = -1/tau; 
        
    
    % controls
    B(3,1)=1/tau;    
    B(4,2)=1;    
    
    C = eye(n);
    D = 0 ;  
    
 
    
    
end 