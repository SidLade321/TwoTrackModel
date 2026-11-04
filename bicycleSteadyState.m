function [delta, ok] = bicycleSteadyState(p, R, v)
% BICYCLESTEADYSTATE  Nonlinear bicycle model held on a circle of radius R.
%   [delta, ok] = bicycleSteadyState(p, R, v)
%
%   Same Pacejka tyres as the two-track model, but each axle's load is
%   split equally between its two tyres (no lateral load transfer).
%   Small-angle steady state:
%       front axle force = m*ay*b/L,  rear axle force = m*ay*a/L
%       delta = L/R + alpha_f - alpha_r
%   ok = false if either axle cannot produce the force needed (grip limit).

ay = v^2 / R;
Fdown = 0.5 * p.rho * p.ClA * v^2;
Fzf = (p.m*p.g*p.b/p.L + Fdown*p.aeroBalF) / 2;
Fzr = (p.m*p.g*p.a/p.L + Fdown*(1 - p.aeroBalF)) / 2;

af = slipFor(p, p.m*ay*p.b/p.L / 2, Fzf);   % per-tyre force needed
ar = slipFor(p, p.m*ay*p.a/p.L / 2, Fzr);
ok = ~isnan(af) && ~isnan(ar);
delta = p.L/R + af - ar;
end

function alpha = slipFor(p, F, Fz)
% Slip angle that gives lateral force F at load Fz (rising side of the curve)
fy = @(al) tyreLateral(p, al, Fz);
alphaPk = fminbnd(@(al) -fy(al), 0, 30*pi/180);   % slip at peak force
if F >= fy(alphaPk)
    alpha = NaN;                                    % beyond the grip limit
else
    alpha = fzero(@(al) fy(al) - F, [0 alphaPk]);
end
end
