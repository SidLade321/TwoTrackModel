function [dx, out] = vehicleODE(t, x, p, steerFn, vref)
% VEHICLEODE  3-DOF two-track vehicle model: states [vx; vy; r].
%   dx = vehicleODE(t, x, p, steerFn, vref)    (for ode45)
%   [dx, out] = ...                             (for post-processing)
%
%   x       : [vx; vy; r]  forward speed [m/s], lateral speed [m/s], yaw rate [rad/s]
%   p       : params()
%   steerFn : function handle, steerFn(t) -> front road-wheel angle [rad], +ve = left
%   vref    : target forward speed [m/s] (simple speed-hold controller on rear drive)
%
% Body frame: x forward, y left, z up. Rear-wheel drive.
% Simplifications (to revisit later):
%   - Load transfer uses quasi-static ay = vx*r (steady-state approximation)
%   - No combined slip: drive force does not reduce lateral grip
%   - Equal steer on both front wheels (no Ackermann)

vx = x(1); vy = x(2); r = x(3);
vx = max(vx, 1.0);                          % avoid slip-angle singularity at standstill
delta = steerFn(t);

%% Longitudinal: speed-hold drive force at rear + aero drag
Fdrag  = 0.5 * p.rho * p.CdA * vx^2;
Fdrive = Fdrag + 2000 * (vref - vx);        % P-controller [N per m/s]
ax_est = (Fdrive - Fdrag) / p.m;

%% Vertical loads [FL FR RL RR]
ay_est = vx * r;
Fz = loadTransfer(p, ax_est, ay_est, vx);

%% Wheel positions relative to CG
xw = [ p.a,      p.a,     -p.b,     -p.b    ];
yw = [ p.tf/2,  -p.tf/2,   p.tr/2,  -p.tr/2 ];
dw = [ delta,    delta,    0,        0      ];   % steer angle per wheel

%% Slip angles: wheel velocity from rigid-body motion, minus steer
vxw = vx - r .* yw;
vyw = vy + r .* xw;
alpha = dw - atan2(vyw, vxw);

%% Tyre forces in wheel frame
Fyw = tyreLateral(p, alpha, Fz);
Fxw = [0, 0, Fdrive/2, Fdrive/2];

%% Rotate into body frame
Fxb = Fxw .* cos(dw) - Fyw .* sin(dw);
Fyb = Fxw .* sin(dw) + Fyw .* cos(dw);

%% Sum forces and yaw moment
Fx = sum(Fxb) - Fdrag;
Fy = sum(Fyb);
Mz = sum(xw .* Fyb - yw .* Fxb);

%% Equations of motion (Newton-Euler in a rotating body frame)
dvx = Fx / p.m + r * vy;
dvy = Fy / p.m - r * vx;
dr  = Mz / p.Izz;
dx = [dvx; dvy; dr];

%% Optional outputs for plotting
if nargout > 1
    out.delta = delta;
    out.alpha = alpha;
    out.Fz    = Fz;
    out.Fy    = Fyw;
    out.ay    = Fy / p.m;               % true lateral acceleration
    out.beta  = atan2(vy, vx);          % body slip angle
end
end
