function v = rot_2d(vector,theta)

    rot = [cos(theta) -sin(theta);
           sin(theta)  cos(theta)];
    try
        v = rot*vector;
    catch
        v = rot*vector';
    end

end

