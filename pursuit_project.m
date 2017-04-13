clear all;

h = .01;
%NUM_ITER = 10000;
TMAX = 150;
i = 2;
t = 0;
THRESHOLD = .11;
CONVERGENCE = false;

target_x = zeros(1,floor(TMAX/h) + 2);
target_y = target_x;

pos_x = zeros(1,floor(TMAX/h) + 2);
pos_y = pos_x;

speedval = zeros(1,2);
chaser_speed = zeros(1,floor(TMAX/h) + 2);

distance = ones(1,floor(TMAX/h) + 2);

dx = 0;
dy = 0;

a = 0; b= 0;

movie = input('Do you want to make a pursuit movie? 1 or 0:');

%hfig = figure(1) ;
if movie
    numframes=length(target_x); 
    A=moviein(numframes); % create the movie matrix 
    set(gca,'NextPlot','replacechildren') ;
    axis equal;
end

[speedval(1),speedval(2)] = funcs(pos_x(i-1), pos_y(i-1), t);

chaser_speed(i-1) = sqrt(speedval(1)^2 + speedval(2)^2);

for t = 0:h:TMAX  
%while (~CONVERGENCE && (t <= TMAX + 2))
    [dx, dy] = RK4_pursuit(pos_x(i-1), pos_y(i-1), t, h);
    pos_x(i) = pos_x(i-1) + dx;
    pos_y(i) = pos_y(i-1) + dy;
    %pos_x(i) = pos_x(i-1) + dx*0.95^t;
    %pos_y(i) = pos_y(i-1) + dy*0.95^t;
    [target_x(i-1), target_y(i-1)] = target_kinematics(t);
    distance(i-1) = sqrt((target_x(i-1)-pos_x(i-1))^2+(target_y(i-1)-pos_y(i-1))^2);
    if movie
        axis([-6,0,-1,50]);
        %axis FILL;
        plot(pos_x(i-1),pos_y(i-1),'o',target_x(i-1),target_y(i-1),'x');
    end
    [speedval(1),speedval(2)] = funcs(pos_x(i), pos_y(i), t);
    chaser_speed(i) = sqrt(speedval(1)^2 + speedval(2)^2); 
    if movie
        A(:,i-1)=getframe;
    end
    if distance(i-1) <= THRESHOLD
        CONVERGENCE = true;
        %disp('distance threshold reached');
    end
    
    i = i + 1;
    %t = t + h;
end

if movie
    movie(A,1,120) % Play the MATLAB movie 
end

[target_x(i-1), target_y(i-1)] = target_kinematics(t+h);
distance(i-1) = sqrt((target_x(i-1)-pos_x(i-1))^2+(target_y(i-1)-pos_y(i-1))^2);

if t < TMAX
    t_conv_x = zeros(1,i-1);
    t_conv_y = zeros(1,i-1);
    c_conv_x = zeros(1,i-1);
    c_conv_y = zeros(1,i-1);
    dist_conv = zeros(1,i-1);
    for n = 1:(i-1)
        t_conv_x(n) = target_x(n);
        t_conv_y(n) = target_y(n);
        c_conv_x(n) = pos_x(n);
        c_conv_y(n) = pos_y(n);
        dist_conv(n) = distance(n);
    end
    figure;
    plot(c_conv_x,c_conv_y);
    hold on;
    plot(t_conv_x,t_conv_y);
    figure;
    plot([0:h:t,t+h], dist_conv);
else
    figure;
    
    plot(pos_x, pos_y);
    hold on;
    title('Positions of Pursuer and Target');
    hold on;
    plot(target_x, target_y);
    %hold on;
    figure;
    plot([0:h:t,t+h], distance);
    hold on;
    title('Distance Between Pursuer and Target vs Time');
    xlabel('Time (s)');
    ylabel('Distance (m)');
    %figure;
    %plot([0:h:t,t+h], chaser_speed);
    %hold on;
    %title('Speed of Pursuer vs Time');
end