function lin = linearise(p, v)
% LINEARISE  Linear lateral/yaw model about straight-line driving at speed v.
%   lin = linearise(p, v)
%
%   States x = [vy; r], input u = delta (front steer) [rad].
%       x_dot = A*x + B*u
%       ay    = Cay*x + Day*u
%   Jacobians are taken numerically (central differences) from vehicleODE,
%   so the linear model inherits every effect in the full model.
%
%   lin.A, lin.B, lin.Cay, lin.Day : state-space matrices
%   lin.eig   : eigenvalues of A [1/s]
%   lin.wn    : natural frequency of the yaw mode [rad/s]
%   lin.zeta  : damping ratio
%   lin.Gr0   : steady-state yaw-rate gain r/delta [1/s]

x0 = [v; 0; 0];
h  = 1e-6;
f  = @(x, d) vehicleODE(0, x, p, @(t) d, v);

A = zeros(2); Cay = zeros(1, 2);
idx = [2 3];                                  % vy and r within [vx vy r]
for k = 1:2
    xp = x0; xp(idx(k)) = xp(idx(k)) + h;
    xm = x0; xm(idx(k)) = xm(idx(k)) - h;
    [dp, op] = f(xp, 0);
    [dm, om] = f(xm, 0);
    A(:, k)  = (dp(2:3) - dm(2:3)) / (2*h);
    Cay(k)   = (op.ay - om.ay) / (2*h);
end
[dp, op] = f(x0,  h);
[dm, om] = f(x0, -h);
B   = (dp(2:3) - dm(2:3)) / (2*h);
Day = (op.ay - om.ay) / (2*h);

lin.A = A; lin.B = B; lin.Cay = Cay; lin.Day = Day; lin.v = v;
lin.eig  = eig(A);
lin.wn   = sqrt(real(prod(lin.eig)));         % = sqrt(det A)
lin.zeta = -real(sum(lin.eig)) / (2*lin.wn);  % = -trace(A) / (2*wn)
xs = -A \ B;                                  % steady-state response to unit steer
lin.Gr0 = xs(2);
end
