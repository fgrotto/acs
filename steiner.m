function [I] = steiner(m, r)
    I = m * (r'*r*eye(3) - r*r');
end