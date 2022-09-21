function [Ao, Bo, Co, D]=getAB_steering_delay(x,kappai, tau, L)
    n=3;
    m=1;
 
    
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
    C = eye(3);
    D = 0 ;  
    
    % Add disturbance dimension
     
    Ao = [A, -B; zeros(1, 3), 0];
    Co = [C, zeros(3, 1)];
    Bo = [B;0];
    Do = D;
    
    
end 