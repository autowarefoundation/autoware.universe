# COMMUNICATION DELAY OBSERVER COMPENSATOR (CDOB) WITH a DISTURBANCE OBSERVER (DOB)

The CDOB and DOB observer-based compensators are intended to mitigate the effects of delayed control and
measurement signals on the system behavior.

Time delay on the signal may cause motion instability or reduce the attainable performance of the controllers. As
such it is essential to alleviate the effects in the practical implementation of the control algorithms.

If the amount of time delay is small, we can design robust controllers that can minimize the effects of the
time delay. However, when the time delay gets more significant, we can do little to design robust controllers. There is
almost no solution other than using the predictive controllers (finite-spectrum assignment) or
time-delay compensators.

The former design methodology requires a good estimation and knowledge of the magnitude of time delays. If the
estimated time delay does not match the real value, or there are time-varying time delays, the time-delay problem may
persist, although we use predictive models.

The CDOB compensators theoretically remove the time delay from the closed-loop system equations allowing to
design controllers as if there is no time delay in any signal channel.

### References

1. Natori, K., 2012, March. A design method of time-delay systems with communication disturbance observer by using Pade
   approximation. In 2012 12th IEEE International Workshop on Advanced Motion Control (AMC) (pp. 1-6). IEEE.
2. Zhang, W., Tomizuka, M., Wei, Y.H., Leng, Q., Han, S. and Mok, A.K., 2015, July. Robust time delay compensation in a
   wireless motion control system with double disturbance observers. In 2015 American Control Conference (ACC) (pp.
   5294-5299). IEEE.
2. Wang, H. and Guvenc, L., 2018. Use of Robust DOB/CDOB Compensation to Improve Autonomous Vehicle Path Following
   Performance in the Presence of Model Uncertainty, CAN Bus Delays and External Disturbances (No. 2018-01-1086).
3. Zhang, W., Tomizuka, M., Wu, P., Wei, Y.H., Leng, Q., Han, S. and Mok, A.K., 2017. A double disturbance observer
   design for compensation of unknown time delay in a wireless motion control system. IEEE Transactions on Control
   Systems Technology, 26(2), pp.675-683.
4. Emirler, M.T., 2015. Advanced Control Systems for Ground Vehicles (Doctoral dissertation, PhD Thesis, İstanbul
   Technical University, İstanbul, Turkey).
