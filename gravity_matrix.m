%% Compute the gravity matrix

G = [diff(U, q(1)); diff(U, q(2)); diff(U, q(3))];
G = simplify(G);