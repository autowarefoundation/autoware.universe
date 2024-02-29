.. _basic_usage:

Basic Usage Example
=====================

The environment can work out of the box without too much customization.

A gym env could be instantiated without any extra arguments. By default, it spawns two agents in the Vegas (IROS 2020) map. You can find the image of the map at  ``gym/f110_gym/envs/maps/vegas.png``. At instantiation, the index of the ego agent in the list of agents is 0.

The agents can be reset by calling the ``reset()`` method using a numpy ndarray of size ``(num_agents, 2)``, where each row represents an agent, and the columns are the ``(x, y)`` coordinate of each agent.

The ``reset()`` and ``step()`` method returns:
    - An *observation* dictionary
    - A *step reward*, which in the current release is the physics timestep used.
    - A *done* boolean indicator, flips to true when either a collision happens or the ego agent finishes 2 laps.
    - An *info* dictionary. Empty in the current release.

The action taken by the ``step()`` function is a numpy ndarray of size ``(num_agents, 2)``, where each row represents an agent's action (indices corresponds to the list of agents), and the columns are control inputs (steering angle, velocity).

A working example can be found in ``examples/waypoint_follow.py``.

The following pseudo code provides a skeleton for creating a simulation loop.

.. code:: python

    import gym
    import numpy as np
    from your_custom_policy import planner # the policy/motion planner that you create

    # instantiating the environment
    racecar_env = gym.make('f110_gym:f110-v0')
    obs, step_reward, done, info = racecar_env.reset(np.array([[0., 0., 0.], # pose of ego
                                                               [2., 0., 0.]])) # pose of 2nd agent
    # instantiating your policy
    planner = planner()

    # simulation loop
    lap_time = 0.

    # loops when env not done
    while not done:
        # get action based on the observation
        actions = planner.plan(obs)

        # stepping through the environment
        obs, step_reward, done, info = racecar_env.step(actions)

        lap_time += step_reward

For a more in-depth example that provides more customization to the environment, see :ref:`custom_usage`.
