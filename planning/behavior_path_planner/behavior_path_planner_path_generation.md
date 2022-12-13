## [WIP] Path Generation

This document explains how to generate path for lane change and avoidance, etc by path_shifer.


### Brief

You can see how this path generation is used in the [README](./README) for lane change and avoidance, etc. The basic concept of the path generation is to shift the reference path in the lateral direction with smooth curves. It is assumed that the path shifting from the reference path is done with the following profiles.



<p align="center">
  <img src="./image/path_shifter.png" width="800">
</p>

This figure explains the constant lateral jerk $l^{'''}(s)$ is applied to induce the lateral shifting. To take into account the limits of lateral acceleration and lateral velocity, zero-jerk time is employed in the figure. In each interval with constant jerk, the shift position $l(s)$ is characterized by a polynomial of the third degree.


### Mathematical Derivation

From simple integral operation, the following analytical equations are derived. They describes the distance $l(t)$ in each time under jerk, acceleration and velocity constraint.

```math
\begin{align}
l_1&= \frac{1}{6}jT_j^3\\[10pt]
l_2&= \frac{1}{6}j T_j^3 + \frac{1}{2} j T_a T_j^2 + \frac{1}{2} j T_a^2 T_j\\[10pt]
l_3&= j  T_j^3 + \frac{3}{2} j T_a T_j^2 + \frac{1}{2} j T_a^2 T_j\\[10pt]
l_4&= j T_j^3 + \frac{3}{2} j T_a T_j^2 + \frac{1}{2} j T_a^2 T_j + j(T_a + T_j)T_j T_v\\[10pt]
l_5&= \frac{11}{6} j T_j^3 + \frac{5}{2} j T_a T_j^2 + \frac{1}{2} j T_a^2 T_j + j(T_a + T_j)T_j T_v \\[10pt]
l_6&= \frac{11}{6} j T_j^3 + 3 j T_a T_j^2 + j T_a^2 T_j + j(T_a + T_j)T_j T_v\\[10pt]
l_7&= 2 j T_j^3 + 3 j T_a T_j^2 + j T_a^2 T_j + j(T_a + T_j)T_j T_v
\end{align}
```

These equations are used to decide a path shape. Furthermore, from these basic equations, the following expression are derived.

#### Maximum Acceeration Computation

```
  const auto T = arclength / speed;
  const auto L = std::abs(shift_length);
  const auto amax = 8.0 * L / (T * T);
```

#### Calculation of Ta, Tj and jerk from acceleration limit

```
  const auto tj = T / 2.0 - 2.0 * L / (acc_limit_ * T);
  const auto ta = 4.0 * L / (acc_limit_ * T) - T / 2.0;
  const auto jerk = (2.0 * acc_limit_ * acc_limit_ * T) / (acc_limit_ * T * T - 4.0 * L);
```

#### Required Time from Jerk and Acceleration Constraint

```
  const double j = std::abs(jerk);
  const double a = std::abs(acc);
  const double l = std::abs(lateral);
  if (j < 1.0e-8 || a < 1.0e-8) {
    return 1.0e10;  // TODO(Horibe) maybe invalid arg?
  }

  // time with constant jerk
  double tj = a / j;

  // time with constant acceleration (zero jerk)
  double ta = (std::sqrt(a * a + 4.0 * j * j * l / a) - 3.0 * a) / (2.0 * j);

  if (ta < 0.0) {
    // it will not hit the acceleration limit this time
    tj = std::pow(l / (2.0 * j), 1.0 / 3.0);
    ta = 0.0;
  }

  const double t_total = 4.0 * tj + 2.0 * ta;
  return t_total;
```