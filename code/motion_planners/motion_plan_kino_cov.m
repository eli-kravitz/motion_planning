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
v_lim = [4 8];
phi_lim = [-pi/4 pi/4];
v_dot_lim = [-5 5];
phi_dot_lim = [-pi/8 pi/8];
n = 20000;
p_goal = 0.05;
eps = 0.5;
r = 0.5;

% Stuff for covariance
Q = zeros(5,5) + diag([0 0 0 0.0005 1e-7]);
P0 = zeros(5,5); % Perfect initial knowledge of state

% Generate Workspace for visualization
v{1} = [5 0.8;6 0.8;6 1.4;5 1.4];
v{2} = [12 2.8;13 2.8;13 3.4;12 3.4];
for i = 1:numel(v)
    v{i} = vertex_arrange(v{i});
end
f1 = road_plotter(v,x_lim,y_lim);

[node,ni,T,time,P] = goal_bias_rrt_kino_cov(q_start,q_goal,...
    p_goal,n,eps,v,x_lim,y_lim,l_car,w_car,v_lim,phi_lim,...
    v_dot_lim,phi_dot_lim,L,r,Q,P0);

path_plotter_kino_cov(f1,node,T,q_start,l_car,w_car,L,P);

% path_name = ['/Users/elikravitz/Desktop/CU_Classes/Current/'...
%     'Algorithmic_Motion_Planning/Final_Project/figs/cov.avi'];
% f2 = road_plotter(v,x_lim,y_lim);
% path_vid_kino_cov(f2,node,T,q_start,l_car,w_car,L,P,path_name)
