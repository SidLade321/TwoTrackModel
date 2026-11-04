function [Fy, D] = tyreLateral(p, alpha, Fz)
% TYRELATERAL  Lateral tyre force, simplified Pacejka with load sensitivity.
%   [Fy, D] = tyreLateral(p, alpha, Fz)
%   D     : peak available lateral force [N] (optional second output)
%   alpha : slip angle [rad] (scalar or array)
%   Fz    : vertical load [N] (scalar or same size as alpha)
%   Fy    : lateral force [N], same sign as alpha
%
% Load sensitivity: the friction coefficient falls as load rises, so
% doubling Fz gives LESS than double the peak grip. This is why lateral
% load transfer costs an axle total grip.

t  = p.tyre;
Fz = max(Fz, 0);                                  % lifted wheel -> no force

mu = t.mu0 * (1 + t.kmu * (Fz - t.Fz0) / t.Fz0);  % load-dependent friction
mu = max(mu, 0);
D  = mu .* Fz;                                    % peak force

Ba = t.B * alpha;
Fy = D .* sin(t.C * atan(Ba - t.E * (Ba - atan(Ba))));
end
