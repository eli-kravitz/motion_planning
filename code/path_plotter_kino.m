function path_plotter_kino(f,node,T,q_start,l_car,w_car,L)

figure(f);
c = 'rgbkcmy';
avg = mean(T(:,end));
desired = 0.1; % Average desired pause time
scale = desired/avg;
for i = 2:numel(node)
    idx1 = find(T(:,1) == node(i-1));
    idx2 = find(T(:,1) == node(i));
    for j = 1:size(q_start,1)
        v1 = rot_2d([-l_car/2 -w_car/2],T(idx1,2*j+3));
        v2 = rot_2d([l_car/2 -w_car/2],T(idx1,2*j+3));
        v3 = rot_2d([l_car/2 w_car/2],T(idx1,2*j+3));
        v4 = rot_2d([-l_car/2 w_car/2],T(idx1,2*j+3));
        shift = [T(idx1,2*j+1)+(L/2)*cos(T(idx1,2*j+3)) ...
            T(idx1,2*j+2)+(L/2)*sin(T(idx1,2*j+3))];
        vert(1,:) = shift+v1';
        vert(2,:) = shift+v2';
        vert(3,:) = shift+v3';
        vert(4,:) = shift+v4';
        p = polyshape(vert(:,1),vert(:,2));
        p_plot = plot(p,'FaceColor',c(j),'FaceAlpha',1,'EdgeAlpha',0);
        plot([T(idx1,2*j+1) T(idx2,2*j+1)],...
            [T(idx1,2*j+2) T(idx2,2*j+2)],c(j),'linewidth',3);
        pause(T(idx1,end)*scale);
    end
    if ~(i == 2 || i == numel(node))
        delete(p_plot);
    end
end

end

