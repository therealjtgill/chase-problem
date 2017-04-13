function [Chaser_x, Chaser_y] = RK4_pursuit(C_x, C_y, t, h)

    %Runge Kutta integration of the pursuit equations
 
    K1 = [0 0 0 0];
    K2 = [0 0 0 0];
    
    %K1(1) = f1(C_x,C_y,t);
    %K2(1) = f2(C_x,C_y,t);
    [K1(1), K2(1)] = funcs(C_x, C_y, t);
    
    %K1(2) = f1(C_x + (h/2)*K1(1),C_y + (h/2)*K2(1), t + h/2);
    %K2(2) = f2(C_x + (h/2)*K1(1),C_y + (h/2)*K2(1), t + h/2);
    [K1(2), K2(2)] = funcs(C_x + (h/2)*K1(1),C_y + (h/2)*K2(1), t + h/2);
    
    %K1(3) = f1(C_x + (h/2)*K1(2),C_y + (h/2)*K2(2), t + h/2);
    %K2(3) = f2(C_x + (h/2)*K1(2),C_y + (h/2)*K2(2), t + h/2);
    [K1(3), K2(3)] = funcs(C_x + (h/2)*K1(2),C_y + (h/2)*K2(2), t + h/2);
    
    %K1(4) = f1(C_x + (h)*K1(3),C_y + (h)*K2(3), t + h);
    %K2(4) = f2(C_x + (h)*K1(3),C_y + (h)*K2(3), t + h);
    [K1(4), K2(4)] = funcs(C_x + (h)*K1(2),C_y + (h)*K2(2), t + h);
    
    Chaser_x = (h/6)*(K1(1) + 2*K1(2) + 2*K1(3) + K1(4));
    Chaser_y = (h/6)*(K2(1) + 2*K2(2) + 2*K2(3) + K2(4));
end

