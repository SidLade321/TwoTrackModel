function bm = bicycleModel(p, v)
% BICYCLEMODEL  Linear single-track ("bicycle") model at speed v.
%   bm = bicycleModel(p, v)
%
%   The textbook reference model: each axle is lumped into one tyre, there
%   is NO lateral load transfer, and tyres are linear. Uses the SAME
%   parameters and tyre model as the two-track model, so any difference
%   between the two comes from model structure only.
%
%   Axle cornering stiffness = 2 x the tyre's initial slope dFy/dalpha
%   (= B*C*D for Pacejka) at the static + aero corner load.
%
%   bm.Cf, bm.Cr : front/rear axle cornering stiffness [N/rad]
%   bm.A, bm.B   : state-space, states [vy; r], input delta
%   bm.Kus       : understeer gradient [rad/(m/s^2)]  (>0 = understeer)
%   bm.Gr0       : steady-state yaw-rate gain r/delta [1/s]
%   bm.eig       : eigenvalues of A [1/s]

m = p.m; a = p.a; b = p.b; L = p.L; Iz = p.Izz;

Fdown = 0.5 * p.rho * p.ClA * v^2;
Fzf = (m*p.g*b/L + Fdown*p.aeroBalF) / 2;          % per front tyre
Fzr = (m*p.g*a/L + Fdown*(1 - p.aeroBalF)) / 2;    % per rear tyre
[~, Df] = tyreLateral(p, 0, Fzf);
[~, Dr] = tyreLateral(p, 0, Fzr);
Cf = 2 * p.tyre.B * p.tyre.C * Df;
Cr = 2 * p.tyre.B * p.tyre.C * Dr;

A = [ -(Cf + Cr)/(m*v),          -(a*Cf - b*Cr)/(m*v) - v;
      -(a*Cf - b*Cr)/(Iz*v),     -(a^2*Cf + b^2*Cr)/(Iz*v) ];
B = [ Cf/m; a*Cf/Iz ];

bm.Cf = Cf; bm.Cr = Cr; bm.A = A; bm.B = B; bm.v = v;
bm.Kus = m/L * (b/Cf - a/Cr);
bm.Gr0 = v / (L + bm.Kus * v^2);
bm.eig = eig(A);
end
