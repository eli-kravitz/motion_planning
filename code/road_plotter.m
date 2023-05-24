function f1 = road_plotter(v,x_lim,y_lim)

f1 = figure; hold on; grid on;

% Plot road
road_vertices = [x_lim(1) y_lim(1);x_lim(2) y_lim(1);...
                 x_lim(2) y_lim(2);x_lim(1) y_lim(2)];
poly = polyshape(road_vertices(:,1),road_vertices(:,2));
p = plot(poly);
p.FaceColor = [0.5 0.5 0.5];
p.FaceAlpha = 1;
p.EdgeAlpha = 0;

% Plot dashes in center of road
c = (y_lim(2)-y_lim(1))/2;
for i = 1:3:(x_lim(2)-x_lim(1))
    dash_vertices = [i-0.5 c-0.15;i+0.5 c-0.15;i+0.5 c+0.15;i-0.5 c+0.15];
    poly = polyshape(dash_vertices(:,1),dash_vertices(:,2));
    p = plot(poly);
    p.FaceColor = [1 1 1];
    p.FaceAlpha = 1;
    p.EdgeAlpha = 0;
end

for i = 1:numel(v)
    for j = 1:4
        x(j) = v{i}(j,1);
        y(j) = v{i}(j,2);
    end
    poly = polyshape(x,y);
    p = plot(poly);
    p.FaceColor = [0 0.4470 0.7410];
    p.FaceAlpha = 1;
    p.EdgeAlpha = 0;
end

axis equal
xlim(x_lim); ylim(y_lim);
f1.OuterPosition = [27 466 1318 410];

end

