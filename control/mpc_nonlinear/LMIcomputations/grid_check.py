import numpy as np
import scipy.io

'''
    Creates a grid and save to matlab
    Vehicle Model states [ey, epsi, v, delta]
'''

ind = 0
low_or_high_grid = ['low', 'high'][ind]

delta_max = np.deg2rad(40)  # maximum steering angle
delta_dot_max = np.deg2rad(40)
delta_input_max = np.deg2rad(40)
vmax = 10

eyaw_max = 30  # degrees
amax = 5
kappa_max = 0.1

if low_or_high_grid == "high":
    gey, geyaw, gv, gdelta, ga, gdeltadot, gdelta_input = 4, 4, 8, 4, 4, 4, 4  # states and controls
    gkappa = 2
    save_name = 'xugrid_check.mat'
else:
    gey, geyaw, gv, gdelta, ga, gdeltadot, gdelta_input = 3, 3, 6, 2, 2, 3, 3  # states and controls
    gkappa = 2
    save_name = 'xugrid_rate.mat'

## STATE GRIDS
ey_grid = np.linspace(-1.0, 1.0, gey)
eyaw_grid = np.linspace(-np.deg2rad(eyaw_max), np.deg2rad(eyaw_max), geyaw)

v_grid = np.linspace(1., vmax, gv)
delta_grid = np.linspace(-delta_max, delta_max, gdelta)
kappa_grid = np.linspace(-kappa_max, kappa_max, gkappa)

## CONTROL GRID
a_grid = np.linspace(-amax, amax, ga)
ddot_grid = np.linspace(-delta_dot_max, delta_dot_max, gdeltadot)
delta_grid_input = np.linspace(-delta_input_max, delta_input_max, gdelta_input)

## UNCERTAINTY GRID
gwa = 1
wa_grid = np.array([0])  # np.linspace(-0.1, 0.1, 3)

## GRID SIZE
tshape = gey * geyaw * gv * gdelta * ga * gdeltadot * gdelta_input * gkappa * gwa  ## total size of the grid, glr=2
xx = np.meshgrid(ey_grid, eyaw_grid, v_grid, delta_grid, kappa_grid, gdelta_input, a_grid, ddot_grid, wa_grid)

xx = [xx[i].flatten() for i, _ in enumerate(xx)]  ## All combination of grids - Meshgrid Flattened
xugrid = np.vstack(xx).T  # first size is the batch size

scipy.io.savemat(save_name, dict(xugrid=xugrid))
