function dt = calc_dt(v0,a,r)

% Roots from quadratic equation 0.5at^2 + v0t - r = 0
c1 = 0.5*a;
c2 = v0;
c3 = -r;
dt = [(-c2+sqrt(c2^2-4*c1*c3))/(2*c1);(-c2-sqrt(c2^2-4*c1*c3))/(2*c1)];

% If complex dt, acceleration causes car to go backwards - bad on highway
real_flag = [imag(complex(dt(1))) == 0;imag(complex(dt(2))) == 0];

% Don't want negative root of quadratic equation
pos_flag = [real(dt(1)) > 0;real(dt(2)) > 0];

% Only want increasing position roots
crit = -v0/a;
if a < 0
    % Concave down, want roots to left of critical point
    increase_flag = [real(dt(1)) < crit;real(dt(2)) < crit];
else
    % Concave up, want roots to right of critical point
    increase_flag = [real(dt(1)) > crit;real(dt(2)) > crit];
end
    
msk = real_flag & pos_flag & increase_flag;
dt = dt(msk);
if isempty(dt)
    dt = NaN;
end

end

