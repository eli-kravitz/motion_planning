function [major,minor,ang] = cov2ellipse(cov)

a = cov(1,1);
b = cov(1,2);
c = cov(2,2);

major = real(sqrt((a+c)/2 + sqrt(((a-c)/2)^2+b^2)));
minor = real(sqrt((a+c)/2 - sqrt(((a-c)/2)^2+b^2)));
ang = atan2(major^2-a,b);

end

