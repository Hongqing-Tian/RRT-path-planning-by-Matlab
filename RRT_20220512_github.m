% *************//generate random map and create a route through it//****
% ********* 2022.5.2 Monday********
% ********* Author  Tian Hongqing********
close all;
clear all;
clc;
addpath('C:\New project\Github'); % add the working path of your own
% ********* Original map generation parameters ********
L = 300; W =200;  % Length and width of the map
n = 9; radius = 20; % obstacle number and radius
step = 100;  % advancing step
Start = [20,20]; Goal = [L-20,W-20]; % starting and goal point
margin = [radius, radius]; % map margin for obstacle generation
% **********obstacle generation************
points(1,:) = margin + rand(1,2)*[L-2*radius,0;0,W-2*radius]; % first random point generation
for i = 2:n  % generate the following n random points
    generate_succ_flag = 0;  % set flag if generation succed
    collison_flag = 0; % collision check between existing points
    while generate_succ_flag == 0  % loop while not succeed
        temp_point = margin + rand(1,2)*[L-2*radius,0;0,W-2*radius]; % generate a random point
        collision_flag = 0; % set collision flag
        start_check = norm(temp_point-Start); % distance measurement
        goal_check = norm(temp_point-Goal); % distance measurement
        if start_check <= step + radius % distance out of the start point + step
            collision_flag = 1;  % collide with start
        else if goal_check <= step + radius % distance out of the start point + step
                collision_flag = 1;   % collide with goal
        else
            for j = 1:i-1 % collision check with the rest points
                dist_check = norm(temp_point-points(j,:)); % distance measurement
                if dist_check <= 2*radius % distance out of the radius
                    collision_flag = 1; % collide with other obstacles
                    break;
                end
            end
        end
        if collision_flag == 0  % if no collision
            generate_succ_flag = 1;  % new obstacle generation success flag
            points(i,:) = temp_point;  % creat a new obstacle point
        end
        end
    end
end
figure(1); % begin to plot in figure 1
rectangle('Position',[0,0,L,W]);  % plot inside rectangle
hold on;
for i = 1:n
    Fill_circle(points(i,1),points(i,2),radius,'k');  % plot all obstacles in black
end
axis equal; % x and y axis equal to  each other
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%*************\\ get the screen image \\************
A=getframe(gcf);  % get the figure 1 screen as a map
imwrite(A.cdata,'C:\New project\Github\temp_map.png'); % save it as temporary map on computer
re_image = imread('C:\New project\Github\temp_map.png');
figure(2); % plot another map
imshow(re_image); % show the saved map as an image
hold on;
image_border = [220,180,1305,860]; % create a border for the map
Veh_start = [300,960]; % start position approximately same as the original map by observation
Veh_goal = [1420,260]; % goal position
plot(300,960,'ro');  % plot the start position
plot(1420,260,'bo'); % plot the goal position
rectangle('Position',[220,180,1305,860],"LineWidth",2,"EdgeColor",'k');  % plot inside rectangle
hold on;
%*************\\ Initialize the RRT tree with the only start point \\************
T.v(1).x = Veh_start(1,1);         % Tree's first node x position as vehicle start point
T.v(1).y = Veh_start(1,2);   % Tree's first node y position
T.v(1).xPrev = Veh_start(1,1);     % Previous node x coordination
T.v(1).yPrev = Veh_start(1,2);     % Previous node y coordination
T.v(1).dist=0;          % distance of current node from start
T.v(1).indPrev = 0;     %  the number of previous node
% ********** RRT parameters ************
star_size = 10; % start and goal size
count=1; % count the searching iteration
max_iter = 1000; % maximal iteration
time_interval = 0.1; % pause time for ploting observation
Img_bin = rgb2gray(re_image); % convert the colored map into a binary one
xL=size(Img_bin,2); % get the length of map
yL=size(Img_bin,1); % get the width of map
%*************\\ Initialize the RRT tree with the only start point \\************
plot(Veh_start(1,1), Veh_start(1,2), 'ro', 'MarkerSize',star_size, 'MarkerFaceColor','g'); %  plot the start point with red green
plot(Veh_goal(1,1), Veh_goal(1,2), 'go', 'MarkerSize',star_size, 'MarkerFaceColor','r'); % plot the goal point with red dot
tic;  % begin to record the time
for iter = 1:max_iter % set the maximal iteration
    x_rand=[xL*rand(1),yL*rand(1)]; % get a sample in the map randomly
    for i = 1:count % measure distance to random point form all the nodes in the RRT tree
        Length_branch(i) = norm([x_rand(1,1)-T.v(i).x , x_rand(1,2)-T.v(i).y]); %sqrt((x_rand(1,1)-T.v(i).x)^2 + (x_rand(1,2)-T.v(i).y)^2);
    end
    Nearest_num = find(Length_branch == min(Length_branch)); % get the nearest neighboring node index from RRT tree
    x_near = [T.v(Nearest_num).x , T.v(Nearest_num).y]; % get the nearest node coordination
    seti = atan2(x_rand(1,2)-x_near(1,2) , x_rand(1,1)-x_near(1,1)); % get the advancing angle
    x_new=[x_near(1,1) + step*cos(seti) , x_near(1,2) + step*sin(seti)]; % get the new node at the advancing angle with a fixed step
    if ~collisionChecking(x_near,x_new, Img_bin) % check if it is collision free
        continue; % continue to creat next sample if collision happened
    end
    count=count+1; % index number increased
    T.v(count).x = x_new(1,1);         % coordinate x data stored in T.v
    T.v(count).y = x_new(1,2);    % coordinate y data stored in T.v
    T.v(count).xPrev = x_near(1,1);     % previous node coordinat x set
    T.v(count).yPrev = x_near(1,2);   % previous node coordinate y set
    T.v(count).dist= step;          % step distance set
    T.v(count).indPrev = Nearest_num;     % previous node num set
    
    %%***************** check if the goal reached \\ *************
    %%***************** within the distance of step to goal \\************
    dist_goal = norm(Veh_goal-x_new); % measure the distance to goal. sqrt((x_G-x_new(1,1))^2 + (y_G-x_new(1,2))^2);
    if dist_goal < step % if within step distance
        break;  % break out and get the route
    end
    %%****************\\ plot the route if goal reached\\******************
    plot(T.v(count).x, T.v(count).y,'ro');  % plot the new node with a round dot.
    plot([x_near(1,1); T.v(count).x], [x_near(1,2); T.v(count).y]); %  plot the line between previous node and new node.
    hold on;
    pause(time_interval); % pause for observation convenience
end
%%*****************//find the route and plot with blue lines\\**************
if iter < max_iter  % get the route within given iterations
    path.pos(1).x = Veh_goal(1); % final path goal x coordinate
    path.pos(1).y = Veh_goal(2); % final path goal y coordinate
    path.pos(2).x = T.v(end).x; % the last searching node x coordinate
    path.pos(2).y = T.v(end).y; % the last searching node y coordinate
    pathIndex = T.v(end).indPrev; % the previous node index of the last node.
    j=0;  % set a index for path
    while 1 % loop
        path.pos(j+3).x = T.v(pathIndex).x; % every node x coordinate by index
        path.pos(j+3).y = T.v(pathIndex).y; % every node y
        pathIndex = T.v(pathIndex).indPrev; % previous node index
        if pathIndex == 1 % if get to the point after start
            break % break out when get the data chain of the route
        end
        j=j+1; % the path index increase
    end  %
    path.pos(end+1).x = Veh_start(1,1); % back to the x coordinate of start point
    path.pos(end).y = Veh_start(1,2);   % get the y coordinat of start point
    for j = 2:length(path.pos) % from beginning+1 to the end of the node, draw backwards
        plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'b', 'Linewidth', 3);
    end
    % print the iteration number, time cost and route length approximately.
    time_cost = toc - time_interval*(count-1); % get the calculation time
    path_length = (j-2)*step + dist_goal;  % caculation of path length
    fprintf('Count_num is =%d\n Time cost is =%f\n Path Length=%f\n', iter, time_cost, path_length);  % 计算路径规划时间。
else
    disp('Error, no path found!'); % if more than maximal iterations, quit.
end

%% sub-fuction zone
%% function of drawing a filled circle.
function Fill_circle(x0,y0,r,color) % drawing a circle of obstacle.
aplha=0:pi/40:2*pi;  % 40 parts each circle
x = x0 + r*cos(aplha); % x coordinate of every part
y = y0 + r*sin(aplha); % y coordinate of every part
plot(x,y,'-'); % plot the segment
fill(x,y,color); % fill the color;
end
%% function of checking collision with border and obstacles
function feasible=collisionChecking(startPose,goalPose,map) % check if collision happens
feasible=true; % assume no collision happens
dir=atan2(goalPose(1)-startPose(1),goalPose(2)-startPose(2)); % drive direction
for r=0:0.5:sqrt(sum((startPose-goalPose).^2)) % every check point step is set to be 0.5
    posCheck = startPose + r.*[sin(dir) cos(dir)]; % caculating each check point
    % if out of the range or collide with obstacles, four integer cornors of the check point position.
    if ~(feasiblePoint(ceil(posCheck),map) && feasiblePoint(floor(posCheck),map) && ...
            feasiblePoint([ceil(posCheck(1)) floor(posCheck(2))],map) && feasiblePoint([floor(posCheck(1)) ceil(posCheck(2))],map))
    feasible=false; % to be sure collision happens
    break; % break out from loop for
    end
    if ~feasiblePoint([floor(goalPose(1)),ceil(goalPose(2))],map) % if coincide with the goal
        feasible=false; % break from the loop
    end
end
end
%%***********sub-function of integer point out-border and obstacle checking  ****
function feasible=feasiblePoint(point,map)
feasible=true; % assume no collision happens
% if out of the border range or collision with obstacle.
if ~(point(1)>=1 &&  point(1)<=size(map,2) && point(2)>=1 && point(2)<=size(map,1) && map(point(2),point(1))==255)
    feasible=false;
end
end

