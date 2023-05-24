function [node,ni,T,time] = goal_bias_rrt_kino(q_start,q_goal,...
    p_goal,n,eps,v,x_lim,y_lim,l_car,w_car,v_lim,phi_lim,...
    v_dot_lim,phi_dot_lim,L,r)

t_start = cputime;

% number of states per robot
p = 5;

% T = [node, parent, x1, y1, theta1, v1, phi1 ... dt]
% q_goal = [x1, y1, theta1, v1, phi1 ...]
T = [0 0];
T = [T q_start 0];

ct = 2;
n_iter = 1;
goal_reached = false;
while n_iter < n
    
    % Generate q_rand
    g = rand(1);
    if g <= p_goal
        point = q_goal;
    else
        point(1,1) = x_lim(1)+(x_lim(2)-x_lim(1))*rand(1);
        point(1,2) = y_lim(1)+(y_lim(2)-y_lim(1))*rand(1);
        point(1,3) = 2*pi*rand(1);
        point(1,4) = v_lim(1)+(v_lim(2)-v_lim(1))*rand(1);
        point(1,5) = phi_lim(1)+(phi_lim(2)-phi_lim(1))*rand(1);
    end
    
    % Find q_near
    d = sqrt(sum((T(:,3:end-4) - repmat(point(1:p-3),size(T,1),1)).^2,2));
    [~,idx] = min(d);
    pt_old = T(idx,3:end-1);
    
    % Generate local trajectory
    [pts,dt] = gen_local_traj(point,pt_old,v_dot_lim,phi_dot_lim,r,5,L);
    if isnan(pts)
        continue;
    end
    
    % Check for validity of trajectory
    traj_valid = validity_check_kino(v,pts,l_car,w_car,y_lim,...
        v_lim,phi_lim,L);
    if traj_valid
        T(ct,:) = [T(ct-1,1)+1 T(idx,1) pts(end,:) dt];
        ct = ct+1;
    end
    
    vec = T(ct-1,3:end-2) - q_goal(1:end-1);
    vec = vec.^2;
    d_to_goal = sqrt(sum(vec(1:2)));
    
    if d_to_goal <= eps
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
