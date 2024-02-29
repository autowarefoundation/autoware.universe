.. _custom_usage:

Customized Usage Example
==========================

For a basic usage example, see :ref:`basic_usage`.

The environment also provides options for customization.

Custom Map
------------

The environment uses a convention that is similar to the ROS map convention. A map for the environment is created by two files: a ``yaml`` file containing the metadata of the map, and a single channel black and white image that represents the map, where black pixels are obstacles and white pixels are free space.

Map Metadata File (yaml)
~~~~~~~~~~~~~~~~~~~~~~~~~~

Only the ``resolution`` and the ``origin`` fields in the yaml files are used by the environment, and only the first two coordinates in the ``origin`` field are used. The unit of the resolution is *m/pixel*. The x and y (first two) numbers in the origin field are used to determine where the origin of the map frame is. Note that these two numbers follow the ROS convention. They represent the **bottom left** corner of the map image's coordinate in the world.

Map Image File
~~~~~~~~~~~~~~~~~~~~~~~~~~

A black and white, single channel image is used to represent free space and obstacles. For example, the Vegas map looks like this:

.. image:: ../gym/f110_gym/envs/maps/vegas.png
    :width: 300
    :align: center

Using a Custom Map
~~~~~~~~~~~~~~~~~~~~~~~~~~

The environment can be instantiated with arguments for a custom map. First, you can place your custom map files (.yaml and the image file) in the same directory, for example ``/your/path/to/map.yaml`` and ``/your/path/to/map.png``. Then you can create the environment with the absolute path to these files like this:

.. code:: python

    env = gym.make('f110_gym:f110-v0',
                   map='/your/path/to/map',
                   map_ext='.png')

The ``map`` argument takes the absolute path to the map files **without** any extensions, and the ``map_ext`` argument takes the extension to the map image file. **Note** that it is assumed that the two files are in the **same directory**, and have the **same filename** (not including the extensions, for example map.png and map.yaml)

Random Track Generator (Beta)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
A random track generator derived from the OpenAI CarRacing environment is also provided in ``gym/f110_gym/unittest/random_trackgen.py``. Note that it requires extra dependencies. ``OpenCV`` is required for image manipulation and IO operations, ``shapely`` and ``matplotlib`` are required for image generation. For OpenCV, you can follow the tutorial at https://docs.opencv.org/master/df/d65/tutorial_table_of_content_introduction.html for installation on different platforms. Then you can install ``shapely`` and ``matplotlib`` with: ``$ pip3 install shapely matplotlib``.

After you've installed the dependencies, you can run the track generator by:

.. code:: bash

    $ python3 random_trackgen.py --seed 12345 --num_maps 1

where the ``--seed`` argument (int, default 123) is for reproducibility, and the ``--num_maps`` argument is for the number of maps you want to generate. By default, the script will create two directories in ``unittest``: ``unittest/maps`` and ``unittest/centerline``. The former contains all the map metadata and map image files, and the latter contains a csv file of the (x, y) coordinates of the points on the centerline of the track.

An example of a randomly generated track:

.. image:: ../examples/example_map.png
    :width: 300
    :align: center

Multiple Agents
-----------------

You can instantiate an environment with any number of agents (default is 2). For example:

.. code:: python
    
    env = gym.make('f110_gym:f110-v0',
                   num_agents=3)

This will create an environment with 3 agents. Note that your call to the ``reset()`` and ``step()`` function will have to change accordingly:

.. code:: python

    _, _, _, _ = env.reset(np.array([[ego_x, ego_y],
                                     [opp1_x, opp1_y],
                                     [opp2_x, opp2_y]]))
    _, _, _, _ = env.step(np.array([[ego_steer, ego_speed],
                                     [opp1_steer, opp1_speed],
                                     [opp2_steer, opp2_speed]]))

Note that performance of the environment will start to degrade as more and more agents are added.

Changing Parameters in Vehicle Dynamics
------------------------------------------

The vehicle dynamic model used in the environment is the Single-Track Model from https://gitlab.lrz.de/tum-cps/commonroad-vehicle-models/.

You can change the default paramters (identified on concrete floor with the default configuration F1TENTH vehicle) used in the environment in two ways.

1. You could instantiate the environment with a parameter dictionary:

.. code:: python

    params_dict = {'mu': 1.0489,
                   'C_Sf': 4.718,
                   'C_Sr': 5.4562,
                   'lf': 0.15875,
                   'lr': 0.17145,
                   'h': 0.074,
                   'm': 3.74,
                   'I': 0.04712,
                   's_min': -0.4189,
                   's_max': 0.4189,
                   'sv_min': -3.2,
                   'sv_max': 3.2,
                   'v_switch':7.319,
                   'a_max': 9.51,
                   'v_min':-5.0,
                   'v_max': 20.0,
                   'width': 0.31,
                   'length': 0.58}
    env = gym.make('f110_gym:f110-v0', params=params_dict)

2. Or you could update the parameters of a specific vehicle in the list of vehicles (or all vehicles):

.. code:: python
        
    # env with default params and 2 agents
    env = gym.make('f110_gym:f110-v0')

    # new params
    params_dict = {'mu': 1.0489,
                   'C_Sf': 4.718,
                   'C_Sr': 5.4562,
                   'lf': 0.15875,
                   'lr': 0.17145,
                   'h': 0.074,
                   'm': 3.74,
                   'I': 0.04712,
                   's_min': -0.4189,
                   's_max': 0.4189,
                   'sv_min': -3.2,
                   'sv_max': 3.2,
                   'v_switch':7.319,
                   'a_max': 9.51,
                   'v_min':-5.0,
                   'v_max': 20.0,
                   'width': 0.31,
                   'length': 0.58}

    # update params of only the 2nd vehicles
    env.update_params(params_dict, index=1)

    # update params of all vehicles
    env.update_params(params_dict)

The dynamic model's physical parameters are:
    - **mu**: surface friction coefficient *[-]*
    - **C_Sf**: Cornering stiffness coefficient, front *[1/rad]*
    - **C_Sr**: Cornering stiffness coefficient, rear *[1/rad]*
    - **lf**: Distance from center of gravity to front axle *[m]*
    - **lr**: Distance from center of gravity to rear axle *[m]*
    - **h**: Height of center of gravity *[m]*
    - **m**: Total mass of the vehicle *[kg]*
    - **I**: Moment of inertial of the entire vehicle about the z axis *[kgm^2]*
    - **s_min**: Minimum steering angle constraint *[rad]*
    - **s_max**: Maximum steering angle constraint *[rad]*
    - **sv_min**: Minimum steering velocity constraint *[rad/s]*
    - **sv_max**: Maximum steering velocity constraint *[rad/s]*
    - **v_switch**: Switching velocity (velocity at which the acceleration is no longer able to create wheel spin) *[m/s]*
    - **a_max**: Maximum longitudinal acceleration *[m/s^2]*
    - **v_min**: Minimum longitudinal velocity *[m/s]*
    - **v_max**: Maximum longitudinal velocity *[m/s]*
    - **width**: width of the vehicle *[m]*
    - **length**: length of the vehicle *[m]*
