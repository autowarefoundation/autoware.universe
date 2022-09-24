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

 ```
 
  /**
   *  Define a transfer function by its numerator and denominator.
   * */
   
  std::vector<double> num{1.};
  std::vector<double> den{5.039e-07, 0.00019, 0.02387, 1};

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

The TF-factor operations are based on the Boost Polynomial Library and provide the addition, subtraction and
multiplication of the transfer functions. 