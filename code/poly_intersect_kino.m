function intersect = poly_intersect_kino(pts,vert1,vert2)

intersect = false;

% Quick check to limit computation time - circle that circumscribes
% rectangle method
d1 = norm(vert1(1,5:6) - vert1(1,1:2))/2;
d2 = norm(vert2(3,:) - vert2(1,:))/2;
c_obs = [vert2(1,1)+(vert2(2,1)-vert2(1,1))/2 ...
    vert2(1,2)+(vert2(3,2)-vert2(1,2))/2];
d = sqrt(sum((pts-c_obs).^2,2));
if all(d > d1+d2)
    return;
end

% Longer check using discretization
step_size = 0.2;
vert2 = [vert2;vert2(1,:)];

for i = 1:size(vert2,1)-1
    edge = vert2(i+1,:) - vert2(i,:);
    pts = [(vert2(i,1)+edge(1)*(0:step_size:1))' ...
        (vert2(i,2)+edge(2)*(0:step_size:1))'];
    for j = 1:size(vert1,1)
        x = vert1(j,1:2:end)';
        y = vert1(j,2:2:end)';
        coll = inpolygon(pts(:,1),pts(:,2),x,y);
        if sum(coll) ~= 0
            intersect = true;
            return;
        end
    end
end

end

