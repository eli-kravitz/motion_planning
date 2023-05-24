clear all; close all; clc;

% Car info
l_car = 1;
w_car = 0.6;
L = 0.7; % axel to axel

% RRT info
q_start = [0 1 0 5 0];
q_goal = [20 1 0 5 0];
x_lim = [q_start(1)-l_car q_goal(1)+l_car];
y_lim = [0 4];
v_lim = [0 8];
phi_lim = [-pi/4 pi/4];
v_dot_lim = [-5 5];
phi_dot_lim = [-pi/8 pi/8];
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

[node,ni,T,time] = goal_bias_rrt_kino(q_start,q_goal,...
    p_goal,n,eps,v,x_lim,y_lim,l_car,w_car,v_lim,phi_lim,...
    v_dot_lim,phi_dot_lim,L,r);

path_plotter_kino(f1,node,T,q_start,l_car,w_car,L);

% path_name = ['/Users/elikravitz/Desktop/CU_Classes/Current/'...
%     'Algorithmic_Motion_Planning/Final_Project/figs/kino.avi'];
% f2 = road_plotter(v,x_lim,y_lim);
% path_vid_kino(f2,node,T,q_start,l_car,w_car,L,path_name);

