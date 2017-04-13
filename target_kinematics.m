function [target_x, target_y, target_vel_x, target_vel_y, target_acc_x, target_acc_y] = target_kinematics(t)
    %target's motion

    pos0 = [-10/sqrt(2) -10/sqrt(2)];
    vel0 = [3 1.0];
    
    %constant-velocity
    %----------------------------------------
    target_x = (pos0(1) + vel0(1)*t);
    target_y = (pos0(2) + vel0(2)*t);
    
    target_vel_x = vel0(1);
    target_vel_y = vel0(2);
    %----------------------------------------
    
    %trochoid
    %----------------------------------------
    %target_x = (pos0(1) + 1*cos(.5*t) + .5*t);
    %target_y = (pos0(2) + 1*sin(.5*t));
    
    %target_vel_x = -.5*sin(.5*t) + 0;
    %target_vel_y =  .5*cos(.5*t);
    %----------------------------------------
    
    %circle
    %----------------------------------------
    %target_x = (pos0(1) + 1*cos(.5*t));
    %target_y = (pos0(2) + 1*sin(.5*t));
    
    %target_vel_x = -.5*sin(.5*t);
    %target_vel_y =  .5*cos(.5*t);
    %----------------------------------------
    
    target_acc_x = 0;
    target_acc_y = 0;
end

