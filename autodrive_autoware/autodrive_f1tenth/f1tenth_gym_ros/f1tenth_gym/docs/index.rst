.. image:: assets/f1_stickers_01.png
  :width: 60
  :align: left

F1TENTH Gym Documentation 
================================================

Overview
---------
The F1TENTH Gym environment is created for research that needs a asynchronous, realistic vehicle simulation with multiple vehicle instances in the same environment, with applications in reinforcement learning.

The environment is designed with determinism in mind. All agents' physics simulation are stepped simultaneously, and all randomness are seeded and experiments can be reproduced. The explicit stepping also enables the physics engine to take advantage of faster than real-time execution (up to 30x realtime) and enable massively parallel applications.

Github repo: https://github.com/f1tenth/f1tenth_gym

Note that the GitHub will have more up to date documentation than this page. If you see a mistake, please contribute a fix!

Example Usecases
------------------

1. The gym environment is used as the backend for the F1TENTH virtual racing online competition at IROS 2020:

.. raw:: html

  <iframe width="560" height="315" src="https://www.youtube.com/embed/VzrbRwhDw_c" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>


2. The gym environment is used as the simulation engine for the FormulaZero project: https://github.com/travelbureau/f0_icml_code

.. raw:: html

    <iframe width="560" height="315" src="https://www.youtube.com/embed/7Yat9FZzE4g" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

3. The gym environment is used as the simulation engine for the TunerCar project: http://www.lewissoft.com/pdf/ICRA2020/1667.pdf

.. raw:: html

  <iframe width="560" height="315" src="https://www.youtube.com/embed/ay7L4VAfa_w" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

Citing
--------
If you find this Gym environment useful, please consider citing:

.. code::
  
  @inproceedings{o2020textscf1tenth,
    title={textscF1TENTH: An Open-source Evaluation Environment for Continuous Control and Reinforcement Learning},
    author={O’Kelly, Matthew and Zheng, Hongrui and Karthik, Dhruv and Mangharam, Rahul},
    booktitle={NeurIPS 2019 Competition and Demonstration Track},
    pages={77--89},
    year={2020},
    organization={PMLR}
  }

Physical Platform
-------------------

To build a physical 1/10th scale vehicle, following the guide here: https://f1tenth.org/build.html

.. image:: assets/f110cover.png
  :width: 400
  :align: center

.. toctree::
  :caption: INSTALLATION
  :maxdepth: 2

  installation


.. toctree::
  :caption: USAGE
  :maxdepth: 2

  basic_usage
  customized_usage

.. toctree::
  :caption: REPRODUCIBILITY
  :maxdepth: 2

  reproduce

.. toctree::
  :caption: API REFERENCE
  :maxdepth: 2

  api/base_classes
  api/dynamic_models
  api/laser_models
  api/collision_models
  api/env
  api/obv
  api/rendering
