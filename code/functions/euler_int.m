function y = euler_int(t,y,u1,u2,L)

step = 20;
for i = 1:step
    y_dot = eom(t/step,y,u1,u2,L);
    y = y + t/step*y_dot';
end

end

