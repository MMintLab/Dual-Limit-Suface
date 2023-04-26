% path_plan_sample.m
% Xili-Yi, 
% MMINT LAB, UNIVERSITY OF MICHIGAN
% sample code of using dual-limit-surface model to plan a slippage-free
% sliding path

clear;
clc;


%% planning sliding path:
% parameters:
initial_pose = [0,0,0].';           %[x_0, y_0, theta_0]
goal_pose = [0.05, 0.6, 1.57].';    %[x_end, y_end, theta_end]
k_v = 1;                            % slope of slippage-free cone
k_a = 1;                            % weight of smoothness term
steps = 30;                         % discretize step number

% plan the path:
[success, waypoints] = path_plan(k_v, initial_pose, goal_pose, steps, k_a, false);

%% visualization:
% plot the path as a quiver plot:
plot_path_quiver(waypoints, false,3);

% quiver animation of the path:
plot_path_quiver(waypoints, true, 3);