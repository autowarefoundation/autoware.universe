# Planning Architecture Rationale

## Requirements for Planning
Plannig architecture must be able to support any functions required to achieve the overall use case stated in [Overview](/Overview.md)

This includes: 
- Calculates route that navigates to the desired goal
- Plans maneuver to follow the route (e.g. when to lane change, when to turn at intersection)
- Make sure that vehicle does not collide with obstacles, including pedestrians and other vehicles)
- Make sure that the vehicle follows traffic rules during the navigation. This includes following traffic light, stopping at stoplines, stopping at crosswalks, etc. 

Also, since Autoware is open source software and is meant to be used/developed by anyone around the world, the architecture must be:
- Architecture is extensible enough to integrate new algorithms without changing the interface
- Architecture is extensible enough to adapt to new traffic rules for different countries

## Considered Architecture

Before designing planning architecture, we have looked into papers including ones from the participants of the DARPA Urban Challenge. We have also investigated the planning architecture of Apollo-Auto(version 5.5).

The summary is explained below.

### Boss
System overview of Boss is explained in this [paper](https://www.ri.cmu.edu/pub_files/pub4/urmson_christopher_2008_1/urmson_christopher_2008_1.pdf)
The planner is decomposed into three layers: mission, behavior, and motion. Mission calculates the high-level global path from starting point to goal point, behavior makes tactical decisions such as lane change decisions and maneuvers at an intersection, and motion calculates low-level trajectory with consideration of kinematic model of the vehicle.

**pros**
* It is intuitive. Data flow is one-directional from mission to behavior to motion. Simlar approach was taken by different teams in the DARPA Urban Challenge.
* It is suitable for OSS used world-wide since all traffic rule handling is done in the behavior layer, and developers only need to modify the behavior layer to support their local rules.
  
**cons** 
* Behavior layer can only make "conservative" decisions. Since the behavior layer has no way of knowing the actual trajectory that is calculated by following motion layer, the behavior layer cannot be certain about the validity of decisions. For example, the behavior planner can command lane change only when it is obvious that there is no obstacle in the target lane, and it is not possible to do "aggressive" lane change as an architecture.

### Unified Behavior and Motion
This [paper](https://www.researchgate.net/publication/315067229_Improved_Trajectory_Planning_for_On-Road_Self-Driving_Vehicles_Via_Combined_Graph_Search_Optimization_Topology_Analysis) reviews planning architecture used in the Darpa Urban Challenge and addresses the demerit of splitting behavior layer and motion layer. It also proposes an algorithm to handle decision making and trajectory optimization simultaneously.

**pros** 
* It can make more "aggressive" decisions compared to BOSS type architecture.

**Cons**
* The paper focuses on lane change, but the real environment requires a variety of decision making, such as traffic lights and crosswalks. It is questionable that we can handle different kinds of traffic rules with one optimization algorithm.
* Also, since decision making is also the target of optimization, it is usually difficult to add user-specific constraints to decision making.

### Victor Tango Type
System overview of Victor Tango is explained in this [paper](https://archive.darpa.mil/grandchallenge/TechPapers/Victor_Tango.pdf).
Victor Tango split Behavior and Motion layer like Boss type. However, unlike BOSS type, there is feedback from motion whether a decision made by behavior layer is achievable or not.
**Pros**
* It overcomes the weakness of Boss type and can consider trajectory at the behavior level

**Cons**
* The interface between behavior and motion would be tight and dense. It has the risk of having heavy inter-dependency as new traffic rules are added, making it difficult to replace one of the modules with new algorithms in the future.

### Apollo
Here is the [link](https://github.com/ApolloAuto/apollo/tree/r5.0.0/modules/planning)
Apollo kept updating the planning module at each version update. In version 5.0, they have taken a different approach from others. Apollo split the behavior planner into scenarios, such as intersection, parking, and lane following. In each scenario module, it calls decider and optimizer libraries to achieve specific scenarios

**Pros**
* Specific parameter tunings are available for different scenarios, making it relatively easier to add new traffic rules for different countries
* Different optimizers can be used for a different purpose

**Cons** 
* As the number of scenario increases, planning selector(or scenario selector) may get very complex as the number of scenario increases

## Autoware Planning Architecture
Considering pros and cons of different approaches, we have concluded to take the hybrid approach of Apollo and Boss. 

We divided planning modules into scenarios just like Apollo, but into a more high-level scenario, such as LaneDriving and Parking. Apollo has smaller units of scenarios, such as intersection and traffic lights, but we expect that those scenarios may occur in parallel(e.g. intsection with traffic lights and crosswalks) and preparing combined-scenario modules would make scenario-selector too complex to be maintained. Instead, we made a scenario to be a more broad concept to keep the number of scenarios to be lower to keep scenario selector comprehensible. Currently, we only have LaneDriving and Parking, and we anticipate to have HighWay and InEmergency scenarios in the future. 

More investigation is required to establish the definition of a “Scenario”, but the convention is that a new scenario must only be added whenever a different "paradigm" is needed for planning. 

For example, LaneDriving is used to drive along public roads, whereas Parking is used for driving free space, and it would be difficult to develop an algorithm to support requirements for both scenarios. LaneDriving wouldn't require complex path planner as the shape of lanes are already given from HD map, but it must be done at a higher frequency to drive at higher speed, whereas parking requires more complex maneuvers with cut-backs but with lower constraint about computation time. Therefore, it would make more sense to split them into different scenarios, and design a planner to support contradicting requirements. 

Note that we didn't split LaneDriving into smaller scenarios, unlike Apollo. We have considered all traffic rule related scenes on public roads, including yielding, traffic lights, crosswalks, and bare intersections, to be essentially velocity planning along lanes and can be handled within a single scheme. 


# Reference
* Boss: https://www.ri.cmu.edu/pub_files/pub4/urmson_christopher_2008_1/urmson_christopher_2008_1.pdf
* CMU Doctor These: https://www.researchgate.net/publication/315067229_Improved_Trajectory_Planning_for_On-Road_Self-Driving_Vehicles_Via_Combined_Graph_Search_Optimization_Topology_Analysis
* Victor Tango: https://archive.darpa.mil/grandchallenge/TechPapers/Victor_Tango.pdf
* Apollo Auto: https://github.com/ApolloAuto/apollo/tree/r5.0.0/modules/planning
