# Autoware Control Toolbox (ACT) Documentation

The ACT contains some essential control components in designing Autoware controllers and control
system-related packages and algorithms. The current release incorporates transfer function, state-space, and spline
modules.

## Transfer Function Representation

It is sometimes easier to design and analyze the control or filtering system model by their Transfer Function
representation.
The ACT-TF module consists of simple transfer function objects (TF factors) equipped with algebraic operations by which
one can
add,
subtract or multiply transfer functions.

```c++

/**
*  Define a transfer function by its numerator and denominator.
* */

std::vector<double> num{ 1. };
std::vector<double> den{ 5.039e-07, 0.00019, 0.02387, 1 };

// Constructor by two vectors.
ns_control_toolbox::tf sys(num, den);

// Print the transfer function representation.
sys.print();

/**
*   Prints
*                1
*   -------------------------------------------
*   5.039e-07 s^3 + 0.00019 s^2 + 0.02387 s + 1
**/

```

The TF-factor operations are based on
the [Boost Polynomial Library](https://www.boost.org/doc/libs/1_67_0/libs/math/doc/html/math_toolkit/polynomials.html)
and provide the addition, subtraction and
multiplication of the transfer functions. If required, the addition, subtraction and polynomial power operations on
a transfer function, we recommend to use the TF-factors first for these operations and construct the transfer
function from the resulting factors. We equipped TF class by only TF-TF multiplication and taking an inverse of them.

Here we provide some algebraic operations on the TF-factors and TF-objects, their construction and usage.

```c++

std::vector<double> a{ 1, 4 };
std::vector<double> b{ 3, 8, 7, 1.2 };

// Create two TF-factors.
ns_control_toolbox::tf_factor tf1(a);
ns_control_toolbox::tf_factor tf2(b);

// Add two TF-factors.
auto tf3 = tf1 + tf2;

ns_utils::print("TF factors summation : ");
print(tf3());

// Subtract two factors
auto tf4 = tf1 - tf2;
ns_utils::print("TF factors subtraction : ");
print(tf4());

// USING tf1*tf2 multiplication
ns_utils::print("\n Multiplication of tf1, tf2 \n");
ns_utils::print("Matlab Results  : 3 s^4 + 20 s^3 + 39 s^2 + 29.2 s + 4.8 \n");

auto tf5 = tf1 * tf2;

ns_utils::print("\n Multiplication of tf1, tf2 \n");
print(tf5());

// Power of a TF-factor.
ns_control_toolbox::tf_factor tf7({ 0.2, 1.1 });
tf7.power(3);

ns_utils::print("\n Power of a TF  \n");
print(tf7());

```

The transfer function class has a form of

$$
y(s) = \frac{a}{b} \frac{num(s)}{den(s)}
$$

where, $\mathnormal{a}$ and $\mathnormal{b}$ are the constant multipliers if exist and $\frac{num(s)}{den(s)}$ is a
transfer function which can be static or dynamically changing. If there are constant and dynamic parts in the
transfer function, we can construct it as given in the following code snippet;

```c++

// sys(s) = tf(num, den, a, b)
ns_control_toolbox::tf tf2{{ 2 }, { 1, 1 }, 3., 3. };

/** prints
 *  2 s + 2
 *  -------
 *  3 s + 1
 * */

```

### PADE APPROXIMATION

Delay in the control and measurement channels is an undeniable reality in practical control applications. It
often requires extra care as it causes instability and unwanted oscillation in the system response. To model delay
in the Autoware control applications, we included the Padé approximation method in the ACT based on the algorithm
given in [1]. In the following code snippet, we show how to obtain a transfer function given the amount of
time-delay and the order of the Pade approximation.

```c++
double Td = 0.01;  // time delay in seconds.
size_t order = 3;  // order of the Pade approximation.

auto tf_delay = ns_control_toolbox::pade(Td, order);
tf_delay.print();

//  - s^3 + 1200 s^2 - 6e+05 s + 1.2e+08
//  --------------------------------------
//    s^3 + 1200 s^2 + 6e+05 s + 1.2e+08

auto ss_sys = ns_control_toolbox::tf2ss(tf_delay);
ss_sys.print();

/** prints
     A :

    [ -1200, -585.9, -457.8]
    [  1024,      0,      0]
    [     0,    256,      0]
    ----------------------------------------
     B :

    [64]
    [ 0]
    [ 0]
    ----------------------------------------
     C :

    [ 37.5,     0, 14.31]
    ----------------------------------------
     D :

    [-1]
 * */

// Test discretization and compare with Matlab.
double Ts = 0.1; // sampling time.

// Print discrete state-space representation
ss_sys.print_discrete_system();

/** prints
 Ad :

 [  -0.9999,  -0.03892, -0.002764]
 [ 0.006183,   -0.9926,   -0.1415]
 [  0.07914,   0.09429,   -0.8115]
 ----------------------------------------
  Bd :

 [0.0003865]
 [  0.01979]
 [   0.2533]
 ----------------------------------------
  Cd :

 [  0.5684, -0.05528,    1.297]
 ----------------------------------------
  Dd :

 [0.8187]
**/
```

### Curve Fitting and Interpolation Modules

[See ...](documentation/SplineModules.md)

References :

1. Golub and Van Loan, Matrix Computations, 4rd edition, Chapter 9., Section 9.3.1 pp 530
2. [Matlab Matrix Balancing algorithm.](http://www.ece.northwestern.edu/local-apps/matlabhelp/techdoc/ref/balance.html)
3. [ssbal](http://www.ece.northwestern.edu/local-apps/matlabhelp/toolbox/control/ref/ssbal.html)
4. James, R., Langou, J. and Lowery, B.R., 2014. On matrix balancing and eigenvector computation. arXiv preprint arXiv:
   1401.5766.
