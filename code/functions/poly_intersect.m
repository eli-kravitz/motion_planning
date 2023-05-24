function intersect = poly_intersect(vert1,vert2,y_lim)

step_size = 0.2;
intersect = false;

vert1 = [vert1;vert1(1,:)];
for i = 1:size(vert1,1)-1
    edge = vert1(i+1,:) - vert1(i,:);
    pts = [(vert1(i,1)+edge(1)*(0:step_size:1))' ...
        (vert1(i,2)+edge(2)*(0:step_size:1))'];
    if any(pts(:,2) > y_lim(2)) || any(pts(:,2) < y_lim(1))
        intersect = true;
        return;
    end
    coll = inpolygon(pts(:,1),pts(:,2),vert2(:,1),vert2(:,2));
    if any(coll)
        intersect = true;
        return;
    end
end

end

