function [f1, f2] = funcs(C_x, C_y, t)
    %pursuit equation

    %pursuer's speed
    k = sqrt(9);
    
    %dt determines how greatly the pursuer leads the target
    dt1 = 1/(k);
    dt2 = 1/(k);
    %dt = 0.96^t;
    
    [t_x, t_y, t_vx, t_vy] = target_kinematics(t);
    
    divisor = sqrt((t_x - C_x + dt1*t_vx)^2 + (t_y - C_y + dt2*t_vy)^2);
    %divisor = 1;

    f1 = k*(t_x - C_x + dt1*t_vx)/divisor;
    f2 = k*(t_y - C_y + dt2*t_vy)/divisor;

    %f1 = speed_c*(t_x - C_x);
    %f2 = speed_c*(t_y - C_y);
end

