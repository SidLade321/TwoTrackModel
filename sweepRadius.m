function S = sweepRadius(p, R, vList)
% SWEEPRADIUS  Two-track steady state on radius R, raising speed until the
% car can no longer hold the circle. Returns an array of steadyState structs.
if nargin < 3, vList = 3:0.02:20; end
S = [];
guess = [];
for v = vList
    ss = steadyState(p, R, v, guess);
    if ~ss.converged, break; end
    S = [S; ss]; %#ok<AGROW>
    guess = [ss.delta; ss.vy];
end
end
