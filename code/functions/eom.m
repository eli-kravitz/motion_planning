function dydt = eom(t,y,u1,u2,L)

dydt = [y(4)*cos(y(3));y(4)*sin(y(3));(y(4)/L)*tan(y(5));u1;u2];

end

