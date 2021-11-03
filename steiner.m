function [I] = steiner(m, r)
    % Use steiner theorem for the given mass m and distance r
    I = m * (r'*r*eye(3) - r*r');
end