function [success, waypoints] = path_plan(k_v, start_pose, goal_pose, steps, k_a, inverse)
% PATH_PLAN plan a slippage-free sliding path
% [SUCCESS, WAYPOINTS] = PATH_PLAN(K_V, S_P, G_P, S, K_A, INVERSE) plan a path
% with parameter K_V, start pose S_P, goal pose G_P of S steps, weight of 
% smoothness term is K_A, 
% 
% if in case b,e INVERSE = false, if case c,d INVERSE = true.


c = 0;
linear_pose = [start_pose(1),linspace(start_pose(1), goal_pose(1),steps-2),goal_pose(1),...
    start_pose(2),linspace(start_pose(2), goal_pose(2),steps-2), goal_pose(2)...
    start_pose(3), linspace(start_pose(3), goal_pose(3),steps-2), goal_pose(3)];

f = - 2 * transpose(linear_pose);

Q_0 = eye(steps * 3);
mat_3_0 = [1 -2 1;-2 4 -2; 1 -2 1];
mat_3 = zeros(steps);
for i = 1 : steps - 2
    mat_3_add = blkdiag(zeros(i-1), mat_3_0, zeros(steps - i - 2));
    mat_3 = mat_3 + mat_3_add;
end


mat_4_0 = [1,-3, 3, -1; -3, 9, -9, 3; 3, -9, 9, -3; -1, 3, -3,1];
mat_4 = zeros(steps);
for i = 1 : steps - 3
    mat_4_add = blkdiag(zeros(i-1), mat_4_0, zeros(steps - i - 3));
    mat_4 = mat_4 + mat_4_add;
end
% Q_3 = blkdiag(mat_4, mat_4, mat_4);
Q_3 = blkdiag(mat_3, mat_3, mat_3);

Q = 2*Q_0 + k_a * Q_3;

H = {};
k = {};
d = {};
for i = 1 : steps - 1
    H_tmp = zeros(3 * steps);
    H_tmp(i, i) = k_v;
    H_tmp(i+1, i+1) = k_v;
    H_tmp(i+1, i) = - k_v;
    H_tmp(i, i+1) = - k_v;
    
    H_tmp(i + steps, i + steps) = k_v;
    H_tmp(i+1 + steps, i+1 + steps) = k_v;
    H_tmp(i+1 + steps, i + steps) = - k_v;
    H_tmp(i + steps, i+1 + steps) = - k_v;
        
    H_tmp(i + 2 * steps, i +  2 * steps) = -1;
    H_tmp(i+1 +  2 * steps, i+1 +  2 * steps) = -1;
    H_tmp(i+1 +  2 * steps, i +  2 * steps) = 1;
    H_tmp(i +  2 * steps, i+1 +  2 * steps) = 1;
    if ~inverse
        H{i} = -H_tmp; %(case c, d) 
    else          
        %     H{i} = H_tmp;  %(case b, e) 
        H{i} = H_tmp;
    end
    k{i} = zeros(steps * 3, 1);
    d{i} = 0;
end

% start pose
Aeq_0_1 = [1, zeros(1, steps * 3 - 1)];
Aeq_0_2 = [zeros(1, steps), 1, zeros(1, 2 * steps - 1)];
Aeq_0_3 = [zeros(1, 2 * steps), 1, zeros(1, steps - 1)];
% goal pose
Aeq_1_1 = [zeros(1, steps - 1), 1, zeros(1, steps * 2)];
Aeq_1_2 = [zeros(1, 2 * steps - 1), 1, zeros(1, steps)];
Aeq_1_3 = [zeros(1, 3 * steps - 1), 1];
% start velocity
Aeq_2_1 = [1,-1, zeros(1, steps * 3 - 2)];
Aeq_2_2 = [zeros(1, steps), 1, -1, zeros(1, 2 * steps - 2)];
Aeq_2_3 = [zeros(1, 2 * steps), 1, -1, zeros(1, steps - 2)];
% goal velocity
Aeq_3_1 = [zeros(1, steps - 2), 1, -1, zeros(1, steps * 2)];
Aeq_3_2 = [zeros(1, 2 * steps - 2), 1, -1, zeros(1, steps)];
Aeq_3_3 = [zeros(1, 3 * steps - 2), 1, -1];


Aeq = [Aeq_0_1; Aeq_0_2; Aeq_0_3; Aeq_1_1; Aeq_1_2; Aeq_1_3;...
    Aeq_2_1; Aeq_2_2; Aeq_2_3; Aeq_3_1; Aeq_3_2; Aeq_3_3;];
beq = [start_pose; goal_pose; zeros(6,1)];



options = optimoptions(@fmincon,'Algorithm','interior-point',...
    'SpecifyObjectiveGradient',true,'SpecifyConstraintGradient',true,...
    'HessianFcn',@(x,lambda)quadhess(x,lambda,Q,H), 'MaxIterations',100000,...
    'MaxFunctionEvaluations',100000);
fun = @(x)quadobj(x,Q,f,c);
nonlconstr = @(x)quadconstr(x,H,k,d);
x0 = transpose(linear_pose); % Column vector
x0(12) = 0.3;
[x,fval,eflag,output,lambda] = fmincon(fun,x0,...
    [],[],Aeq,beq,[],[],nonlconstr,options);

x_x = x(1:steps);
x_y = x(steps + 1 : 2 * steps);
x_theta = x(2 * steps + 1 : end);

if length(x)==3*steps
    success = true;
else
    success = false;
end

waypoints = [x_x,x_y,x_theta];
end


function [y,grady] = quadobj(x,Q,f,c)
y = 1/2*x'*Q*x + f'*x + c;
if nargout > 1
    grady = Q*x + f;
end
end


function [y,yeq,grady,gradyeq] = quadconstr(x,H,k,d)
jj = length(H); % jj is the number of inequality constraints
y = zeros(1,jj);
for i = 1:jj
    y(i) = 1/2*x'*H{i}*x + k{i}'*x + d{i};
end
yeq = [];
    
if nargout > 2
    grady = zeros(length(x),jj);
    for i = 1:jj
        grady(:,i) = H{i}*x + k{i};
    end
end
gradyeq = [];
end

function hess = quadhess(x ,lambda,Q,H)
hess = Q;
jj = length(H); % jj is the number of inequality constraints
for i = 1:jj
    hess = hess + lambda.ineqnonlin(i)*H{i};
end
end