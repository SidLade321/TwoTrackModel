function ss = steadyState(p, R, v, guess)
% STEADYSTATE  Steady-state cornering on a constant radius.
%   ss = steadyState(p, R, v)
%   ss = steadyState(p, R, v, guess)   guess = [delta; vy] warm start
%
%   Finds the steer angle delta and lateral velocity vy that hold the car
%   on a circle of radius R [m] at forward speed v [m/s], i.e. with yaw
%   rate r = v/R and zero lateral and yaw acceleration.
%
%   Solved with Newton-Raphson (finite-difference Jacobian) on the lateral
%   and yaw equations of vehicleODE. Longitudinal balance is not enforced
%   (speed is assumed held by the driver).
%
%   ss.converged  : true if an equilibrium was found with all wheels loaded
%   ss.delta      : steer angle [rad]
%   ss.beta       : body slip angle [rad]
%   ss.ay         : lateral acceleration [m/s^2]
%   ss.Fz, ss.alpha, ss.Fy : per-wheel loads, slip angles, forces [FL FR RL RR]
%   ss.utilF, ss.utilR     : axle grip utilisation = sum|Fy| / sum(peak Fy)

r = v / R;
if nargin < 4 || isempty(guess)
    guess = [p.L / R; p.b * r];              % kinematic steer, zero rear slip
end
u = guess(:);

wstate = warning('off', 'loadTransfer:wheelLift');  % trial points may lift a wheel
cleanup = onCleanup(@() warning(wstate));

res = @(u) residual(p, v, r, u);
ss.converged = false;
for it = 1:50
    F = res(u);
    if norm(F) < 1e-8
        ss.converged = true;
        break
    end
    J = zeros(2);
    h = 1e-7;
    for k = 1:2
        du = u; du(k) = du(k) + h;
        J(:,k) = (res(du) - F) / h;
    end
    if rcond(J) < 1e-14, break; end
    u = u - J \ F;
end

[~, o] = vehicleODE(0, [v; u(2); r], p, @(t) u(1), v);
if any(o.Fz < 0), ss.converged = false; end   % model invalid if a wheel lifts

ss.delta = u(1);
ss.vy    = u(2);
ss.beta  = atan2(u(2), v);
ss.ay    = v * r;
ss.v     = v;
ss.Fz    = o.Fz;
ss.alpha = o.alpha;
ss.Fy    = o.Fy;
ss.utilF = sum(abs(o.Fy(1:2))) / sum(o.Fypk(1:2));
ss.utilR = sum(abs(o.Fy(3:4))) / sum(o.Fypk(3:4));
end

function F = residual(p, v, r, u)
dx = vehicleODE(0, [v; u(2); r], p, @(t) u(1), v);
F = dx(2:3);            % lateral and yaw accelerations must be zero
end
