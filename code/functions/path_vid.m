function path_vid(f,node,T,q_start,l_car,w_car,path_name)

figure(f);
outputVideo = VideoWriter(path_name);
outputVideo.Quality = 100;
outputVideo.FrameRate = 5;
open(outputVideo);
c = 'rgbkcmy';
for i = 2:numel(node)
    idx1 = find(T(:,1) == node(i-1));
    idx2 = find(T(:,1) == node(i));
    for j = 1:size(q_start,1)
        vert = [T(idx1,2*j+1)-l_car/2 T(idx1,2*j+2)-w_car/2;...
                T(idx1,2*j+1)+l_car/2 T(idx1,2*j+2)-w_car/2;...
                T(idx1,2*j+1)+l_car/2 T(idx1,2*j+2)+w_car/2;...
                T(idx1,2*j+1)-l_car/2 T(idx1,2*j+2)+w_car/2];
        v1 = rot_2d([-l_car/2 -w_car/2],T(idx1,2*j+3));
        v2 = rot_2d([l_car/2 -w_car/2],T(idx1,2*j+3));
        v3 = rot_2d([l_car/2 w_car/2],T(idx1,2*j+3));
        v4 = rot_2d([-l_car/2 w_car/2],T(idx1,2*j+3));
        vert(1,:) = [T(idx1,2*j+1) T(idx1,2*j+2)]+v1';
        vert(2,:) = [T(idx1,2*j+1) T(idx1,2*j+2)]+v2';
        vert(3,:) = [T(idx1,2*j+1) T(idx1,2*j+2)]+v3';
        vert(4,:) = [T(idx1,2*j+1) T(idx1,2*j+2)]+v4';
        p = polyshape(vert(:,1),vert(:,2));
        p_plot = plot(p,'FaceColor',c(j),'FaceAlpha',1,'EdgeAlpha',0);
        plot([T(idx1,2*j+1) T(idx2,2*j+1)],...
            [T(idx1,2*j+2) T(idx2,2*j+2)],c(j),'linewidth',3);
    end
    writeVideo(outputVideo, getframe(gcf));
    if ~(i == numel(node))
        delete(p_plot);
    end
end
close(outputVideo);

end

