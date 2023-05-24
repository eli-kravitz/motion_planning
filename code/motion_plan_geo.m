clear all; close all; clc;

% Car info
l_car = 1;
w_car = 0.6;

% RRT info
R = 0.5;
q_start = [0 1 0];
q_goal = [20 1 0];
x_lim = [q_start(1)-l_car q_goal(1)+l_car];
y_lim = [0 4];
n = 7500;
p_goal = 0.05;
eps = 0.25;
r = 0.5;

% Generate Workspace for visualization
v{1} = [5 0.8;6 0.8;6 1.4;5 1.4];
v{2} = [12 2.8;13 2.8;13 3.4;12 3.4];
for i = 1:numel(v)
    v{i} = vertex_arrange(v{i});
end
f1 = road_plotter(v,x_lim,y_lim);

[node,ni,T,time] = goal_bias_rrt(q_start,q_goal,r,...
    p_goal,n,eps,v,x_lim,y_lim,l_car,w_car);

path_plotter(f1,node,T,q_start,l_car,w_car);

% path_name = ['/Users/elikravitz/Desktop/CU_Classes/Current/'...
%     'Algorithmic_Motion_Planning/Final_Project/figs/geo.avi'];
% f2 = road_plotter(v,x_lim,y_lim);
% path_vid(f2,node,T,q_start,l_car,w_car,path_name);
