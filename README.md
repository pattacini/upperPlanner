# reaching-planner
===
## Synopsis
- 'reaching-planner' is a solution to generate a motion planner for an arm of the iCub, based on the Incremental Sampling-based Algorithm (RRT*)

- It generates the path plan for the whole upper body of iCub, which includes the End-effector and the mid point in the Forearm (middle of Forearm), by rpc command.

## Commands
- By sending commands through yarp rpc port, we can trigger the planner to generate a plan based on the environment perception
i.e: `yarp rpc /reaching-planner/rpc:i`
- Command for replan
	+`replan <deadline>`, with <deadline> is the maximum exploration time of each planner
 
