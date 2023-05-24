function [node,ni,T,time] = goal_bias_rrt(q_start,q_goal,r,...
    p_goal,n,eps,v,x_lim,y_lim,l_car,w_car)

t_start = cputime;
np = 30;

% number of robots/states per robot
m = size(q_start,1);
p = 3;

% T = [node, parent, x1, y1, theta1, x2, y2, theta2, ...]
% q_goal = [x1, y1, theta1, x2, y2, theta2 ...]
T = [0 0];
q_g = [];
for i = 1:m
    T = [T q_start(i,:)];
    q_g = [q_g q_goal(i,:)];
end
q_goal = q_g;

ct = 2;
n_iter = 1;
goal_reached = false;
while n_iter < n
    
    % Generate q_rand
    g = rand(1);
    if g <= p_goal
        point = q_goal;
    else
        count = 1;
        for i = 1:m
            point(1,count) = x_lim(1)+(x_lim(2)-x_lim(1))*rand(1);
            point(1,count+1) = y_lim(1)+(y_lim(2)-y_lim(1))*rand(1);
            point(1,count+2) = 2*pi*rand(1);
            count = count + p;
        end   
    end
    
    % Find q_near
    d = sqrt(sum((T(:,3:end-1) - repmat(point(1:p-1),size(T,1),1)).^2,2));
    [~,idx] = min(d);
    pt_old = T(idx,3:end);
    
    % Generate q_new points based on step size and vector to random node
    vec_new = point - pt_old;
    vec_new = (vec_new(1:p-1)./norm(vec_new(1:p-1)))*r;
    pts = zeros(np,m*p);
    for i = 1:numel(vec_new)
        pts(:,i) = linspace(pt_old(i),pt_old(i)+vec_new(i),np);
    end
    pts(:,p) = linspace(pt_old(i+1),pt_old(i+1)+point(end),np);
    
    % Check for collisions with obstacles
    in_shape = zeros(np,numel(v)*numel(1:p:m*p));
    count = 1;
    for j = 1:p:m*p
        for l = 1:size(pts,1)
            v1 = rot_2d([-l_car/2 -w_car/2],pts(l,3));
            v2 = rot_2d([l_car/2 -w_car/2],pts(l,3));
            v3 = rot_2d([l_car/2 w_car/2],pts(l,3));
            v4 = rot_2d([-l_car/2 w_car/2],pts(l,3));
            vert(1,:) = pts(l,1:2)+v1';
            vert(2,:) = pts(l,1:2)+v2';
            vert(3,:) = pts(l,1:2)+v3';
            vert(4,:) = pts(l,1:2)+v4';
            for k = 1:numel(v)
                in_shape(count,k) = poly_intersect(vert,v{k},y_lim);
            end
            count = count+1;
        end
    end
    
    if all(all(~in_shape))
        T(ct,:) = [T(ct-1,1)+1 T(idx,1) pts(end,:)];
        ct = ct+1;
    end

    vec = T(ct-1,3:end-1) - q_goal(1:end-1);
    vec = vec.^2;
    for i = 1:m
        d_to_goal(i) = sqrt(sum(vec(i*2-1:i*2)));
    end
    
    if max(d_to_goal) <= eps
        goal_reached = true;
        break;
    end
    
    n_iter = size(T,1);
    
end

if goal_reached
    j = 2;
    path_found = false;
    parent = T(end,2);
    node = T(end,1);
    while ~path_found
        msk = T(:,1) == parent;
        node(j,1) = T(msk,1);
        parent = T(msk,2);
        j = j+1;
        if node(end) == 0
            break;
        end
    end
    node = flip(node);
    ni = n_iter;
else
    node = NaN;
    ni = n_iter;
    time = cputime - t_start;
    return;
end

time = cputime - t_start;

end

