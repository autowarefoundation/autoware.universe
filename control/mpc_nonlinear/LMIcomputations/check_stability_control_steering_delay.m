function [num_of_unstable, num_of_stable] =
check_stability_steering_delay(xugrid, Xrr, Yrr, tau_steering, L, dt
)

[grow, gcol] =
size(xugrid);
num_of_unstable = 0;
num_of_stable = 0;

for
i = 1
:
grow
  ey = xugrid(i, 1);
%
lateral error
epsi = xugrid(i, 2);
%
heading error
vi = xugrid(i, 3);
%
extract speed
deltai = xugrid(i, 4);
%
extract steering
kappai = xugrid(i, 5);
%
curvature grid
usteeri = xugrid(i, 6);
%
curvature grid

uai = xugrid(i, 7);
%
extract acceleration
input
  udi = xugrid(i, 8);
%
extract delta
input
  w2i = xugrid(i, 9);
%
extract input
uncertainty for acc [-0.1, 0.1]

%
Collect x, u
x = [ey;
epsi;
vi;
deltai];
u = [usteeri;
uai;
udi];

%
calculate Y
and X
for
every point
depending on
the parameters
at that
point
[t1, t2, t3, t4] =
GetThetas(x, kappai, L
);

%
Get Time
Derivatives of
Thetas

[A, B, C, D] =
getAB_control_steering_delay(x, kappai, tau_steering, L
);
sys_cont = ss(A, B, C, D);
sys_disc = c2d(sys_cont, dt, 'tustin');

Ad = sys_disc.A;
Bd = sys_disc.B;

Y = Yrr
{ 5 }
+
t1 *Yrr{1}
+
t2 *Yrr{2}
+
t3 *Yrr{3}
+
t4 *Yrr{4};
X = Xrr
{ 5 }
+
t1 *Xrr{1}
+
t2 *Xrr{2}
+
t3 *Xrr{3}
+
t4 *Xrr{4};

%
Pf = inv(Xr);
Kf = Y / X;
%
K = Y * Pf;

Acls = Ad + Bd * Kf;
eigA = eig(Acls);

%
Pdot = -Pf * Xr_dot * Pf;
%
Vdot = Acls
' * Pf + Pf * Acls + Pdot;

eigCond = (abs(eigA(1
:3))<=1);

if
all(eigCond)
%
disp(
'controller is stable')
num_of_stable = num_of_stable + 1;
else
%
disp(
'Not STABLE')
num_of_unstable = num_of_unstable + 1;
end

  end

end