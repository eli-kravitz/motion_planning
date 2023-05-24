function [pts,dt] = gen_local_traj(point,pt_old,v_dot_lim,...
    phi_dot_lim,r,n,L)

opts = odeset('RelTol',1e-2,'AbsTol',1e-2);

u1 = zeros(n,1);
u2 = zeros(n,1);
d = zeros(n,1);
dt = zeros(n,1);
for i = 1:n
    
    u1(i,1) = v_dot_lim(1)+(v_dot_lim(2)-v_dot_lim(1))*rand(1);
    u2(i,1) = phi_dot_lim(1)+(phi_dot_lim(2)-phi_dot_lim(1))*rand(1);
    
    % Calculate dt based on basic kinematics to have traveled distance
    % about equal to r
    dt(i,1) = calc_dt(pt_old(4),u1(i),r);
    
    if ~isnan(dt(i))
        % Use Euler here for speed (rough check), 4th order RK later
        y = euler_int(dt(i),pt_old,u1(i),u2(i),L);
        d(i,1) = norm(y(1:2) - point(1:2));
%         [~,y] = ode45(@(t,y) eom(t,y,u1(i),u2(i),L),...
%             0:dt(i)/2:dt(i),pt_old,opts);
%         d(i,1) = norm(y(end,1:2) - point(1:2));
    else
        d(i,1) = NaN;
    end
    
end

if all(isnan(d))
    pts = NaN;
    return;
end

% Use best controls to define states
[~,idx] = min(d);
[t,pts] = ode45(@(t,y) eom(t,y,u1(idx),u2(idx),L),...
    0:dt(idx)/5:dt(idx),pt_old,opts);
dt = t(end)-t(1);

end
