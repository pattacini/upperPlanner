# reaching-planner
===
## Synopsis
- **`reaching-planner`** is a solution to generate a motion plan for an arm of the iCub, based on the Incremental Sampling-based Algorithm - RRT* [1], [2]

- A path plan includes the End-effector and the mid point in the Forearm (middle of Forearm), generated by trigger from rpc command.

## Dependencies
- [YARP](https://github.com/robotology/yarp)
- [iCub](https://github.com/robotology/icub-main)
- [icub-contrib-common](https://github.com/robotology/icub-contrib-common)
- [WYSIWYD](https://github.com/robotology/wysiwyd): for running with real robot 

## Pipeline

## Commands
- By sending commands through yarp rpc port, we can trigger the planner to generate a plan based on the environment perception,
i.e: `yarp rpc /reaching-planner/rpc:i`

- Command for replan: `replan <deadline>`, with `<deadline>` is the maximum exploration time of each planner

## Documentation
Online documentation is available here: [http://robotology-playground.github.com/reaching-planner](http://robotology-playground.github.com/reaching-planner).

## License
Material included here is Copyright of *iCub Facility - Istituto Italiano di Tecnologia*
and is released under the terms of the GPL v2.0 or later. See the file LICENSE for details.

## Reference
[1] S. Karaman and E. Frazzoli, *"Optimal kinodynamic motion planning using incremental sampling-based methods,"* 49th IEEE Conference on Decision and Control (CDC), Atlanta, GA, 2010, pp. 7681-7687.

[2] S. Karaman and E. Frazzoli, *"Incremental Sampling-based Algorithms for Optimal Motion Planning,"* Proceedings of Robotics: Science and Systems, Zaragoza, Spain, 2010.
 
