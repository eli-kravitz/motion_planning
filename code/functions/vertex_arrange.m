function vertex = vertex_arrange(vertex)
    
    % Arrange vertices bottom left as the first one
    
    % Start by arranging vertices ccw
    % *THIS ONLY WORKS FOR CONVEX POLYGONS*
    center = [mean(vertex(:,1)),mean(vertex(:,2))];
    for i = 1:size(vertex,1)
        vector = center - vertex(i,:);
        % Calculate angle between vector and x axis
        if vector(2) < 0
            theta(i) = 2*pi-acos(dot(vector,[1 0])/norm(vector));
        else
            theta(i) = acos(dot(vector,[1 0])/norm(vector));
        end
    end
    [~,idx] = sort(theta);
    vertex = vertex(idx,:);
    
    % First check for minimum y
    ymin = min(vertex(:,2));
    idx_y = find(vertex(:,2) == ymin);
    
    % If more than one minimum y, choose furthest left
    if numel(idx_y) > 1
        tmp = vertex(:,1);
        tmp = tmp(idx_y);
        xmin = min(tmp);
        idx_x = find(tmp == xmin);
        row = idx_y(idx_x);
    else
        row = idx_y;
    end
    
    % Rearrage vertices
    vertex_tmp = vertex(row:end,:);
    if row ~= 1
        for i = 1:row-1
            vertex_tmp = [vertex_tmp;vertex(i,:)];
        end
    end
    
end

