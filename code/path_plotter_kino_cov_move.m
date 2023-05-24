function path_plotter_kino_cov_move(f,node,T,q_start,l_car,w_car,L,P,vel,v)

figure(f);
c = 'rgbkcmy';
avg = mean(T(:,end));
desired = 0.2; % Average desired pause time
scale = desired/avg;
t_tot = 0;
for i = 2:numel(node)
    
    % Get total time
    idx = find(T(:,1) == node(i));
    t_tot = t_tot+T(idx,end);
    
    % Plot obstacles
    for j = 1:numel(v)
        obs = [v{j}(:,1)+vel(j)*t_tot v{j}(:,2)];
        poly = polyshape(obs(:,1),obs(:,2));
        o(j) = plot(poly);
        o(j).FaceColor = [0 0.4470 0.7410];
        o(j).FaceAlpha = 1;
        o(j).EdgeAlpha = 0;
        
        % Plot line of obstacle
        center_start = [(v{j}(2,1)+v{j}(1,1))/2 (v{j}(3,2)+v{j}(2,2))/2];
        center_new = [(obs(2,1)+obs(1,1))/2 (obs(3,2)+obs(2,2))/2];
        plot([center_start(1) center_new(1)],...
            [center_start(2) center_new(2)],'linewidth',3,...
            'color',[0 0.4470 0.7410]);
    end
    
    idx1 = find(T(:,1) == node(i-1));
    idx2 = find(T(:,1) == node(i));
    for j = 1:size(q_start,1)
        
        % Plot car
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
        p_plot.DisplayName = 'Estimated State';
        
        % Plot line of path
        plot([T(idx1,2*j+1) T(idx2,2*j+1)],...
            [T(idx1,2*j+2) T(idx2,2*j+2)],c(j),'linewidth',3);
        
        % Plot covariance
        if idx1 ~= 1
            cov = P{idx1};
            cov = cov(1:2,1:2);
            [major,minor,theta] = cov2ellipse(cov);
            major = major*3;
            minor = minor*3;
            x0 = T(idx1,3);
            y0 = T(idx1,4);
            t = linspace(0,2*pi,1000);
            x = x0+major*cos(t)*cos(theta)-minor*sin(t)*sin(theta);
            y = y0+minor*sin(t)*cos(theta)+major*cos(t)*sin(theta);
            h = patch(x,y,'g','EdgeAlpha',0,'FaceAlpha',0.2);
            h.DisplayName = 'Covariance';
            
            % Plot all car states
            x = T(idx1,2*j+1)+major*cos(theta);
            y = T(idx1,2*j+2)+major*sin(theta);
            shift = [x+(L/2)*cos(T(idx1,2*j+3)) ...
                y+(L/2)*sin(T(idx1,2*j+3))];
            vert2(1,:) = shift+v1';
            vert2(2,:) = shift+v2';
            vert2(3,:) = shift+v3';
            vert2(4,:) = shift+v4';
            p = polyshape(vert2(:,1),vert2(:,2));
            p_plot2 = plot(p,'FaceColor',c(j),'FaceAlpha',0.2,...
                'EdgeAlpha',0);
            p_plot2.DisplayName = 'Worst Case States'; 
            
            x = T(idx1,2*j+1)-major*cos(theta);
            y = T(idx1,2*j+2)-major*sin(theta);
            shift = [x+(L/2)*cos(T(idx1,2*j+3)) ...
                y+(L/2)*sin(T(idx1,2*j+3))];
            vert3(1,:) = shift+v1';
            vert3(2,:) = shift+v2';
            vert3(3,:) = shift+v3';
            vert3(4,:) = shift+v4';
            p = polyshape(vert3(:,1),vert3(:,2));
            p_plot3 = plot(p,'FaceColor',c(j),'FaceAlpha',0.2,...
                'EdgeAlpha',0); 
            
            x = T(idx1,2*j+1)+minor*cos(pi/2+theta);
            y = T(idx1,2*j+2)+minor*sin(pi/2+theta);
            shift = [x+(L/2)*cos(T(idx1,2*j+3)) ...
                y+(L/2)*sin(T(idx1,2*j+3))];
            vert4(1,:) = shift+v1';
            vert4(2,:) = shift+v2';
            vert4(3,:) = shift+v3';
            vert4(4,:) = shift+v4';
            p = polyshape(vert4(:,1),vert4(:,2));
            p_plot4 = plot(p,'FaceColor',c(j),'FaceAlpha',0.2,...
                'EdgeAlpha',0); 
            
            x = T(idx1,2*j+1)-minor*cos(pi/2+theta);
            y = T(idx1,2*j+2)-minor*sin(pi/2+theta);
            shift = [x+(L/2)*cos(T(idx1,2*j+3)) ...
                y+(L/2)*sin(T(idx1,2*j+3))];
            vert5(1,:) = shift+v1';
            vert5(2,:) = shift+v2';
            vert5(3,:) = shift+v3';
            vert5(4,:) = shift+v4';
            p = polyshape(vert5(:,1),vert5(:,2));
            p_plot5 = plot(p,'FaceColor',c(j),'FaceAlpha',0.2,...
                'EdgeAlpha',0); 
        end
        
        pause(T(idx1,end)*scale);
        
    end
  
    if ~(i == 2 || i == numel(node))
        delete(p_plot);
        delete(h);
        delete(p_plot2);
        delete(p_plot3);
        delete(p_plot4);
        delete(p_plot5);
    end
    if ~(i == numel(node))
        delete(o);
    end
end

if exist('p_plot','var')
    legend([p_plot,p_plot2,h]);
end

end

