function traj_valid = validity_check_kino_move(v,pts,l_car,w_car,y_lim,...
    v_lim,phi_lim,L,dt,vel)

% Velocity and wheel angle checks
if any(pts(:,4) < v_lim(1)) || any(pts(:,4) > v_lim(2))
    traj_valid = false;
    return;
end
if any(pts(:,5) < phi_lim(1)) || any(pts(:,5) > phi_lim(2))
    traj_valid = false;
    return;
end

% Get all vertices for all states to check
in_shape = zeros(numel(v),1);
v1 = rot_2d([-l_car/2 -w_car/2],pts(:,3));
v2 = rot_2d([l_car/2 -w_car/2],pts(:,3)); 
v3 = rot_2d([l_car/2 w_car/2],pts(:,3));
v4 = rot_2d([-l_car/2 w_car/2],pts(:,3));
v1 = reshape(v1,numel(v1)/2,2);
v2 = reshape(v2,numel(v2)/2,2);
v3 = reshape(v3,numel(v3)/2,2);
v4 = reshape(v4,numel(v4)/2,2);
shift = [pts(:,1)+(L/2)*cos(pts(:,3)) pts(:,2)+(L/2)*sin(pts(:,3))];
vert(:,1:2) = shift+v1;
vert(:,3:4) = shift+v2;
vert(:,5:6) = shift+v3;
vert(:,7:8) = shift+v4;

% Bounds check
if min(min(vert(:,2:2:end))) < y_lim(1) || ...
        max(max(vert(:,2:2:end))) > y_lim(2)
    traj_valid = false;
    return;
end

% Check for intersection with obstacles
for k = 1:numel(v)
    v_move = [v{k}(:,1)+vel(k)*dt v{k}(:,2)];
    in_shape(k,1) = poly_intersect_kino(pts(:,1:2),vert,v_move);
end

% Passed all checks if all states not in obstacle
if all(~in_shape)
    traj_valid = true;
else
    traj_valid = false;
end

end

