Collision Models
========================================

This file contains all the numba just-in-time compiled functions for collision checking between agents. The GJK algorithm (more detail here: https://cse442-17f.github.io/Gilbert-Johnson-Keerthi-Distance-Algorithm/) is used to check for overlap in polygons.

.. doxygenfile:: collision_models.py
    :project: f1tenth_gym