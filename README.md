# Multi-armed Bandit Based Model Selection For Deformable Object Manipulation

This repository contains several catkin based packages for testing models and MAB methods for deformable object manipulation; first presented at the 12th International Workshop on the Algorithmic Foundations of Robotics (WAFR 2016), an extended version is available [here](https://arxiv.org/abs/1703.10254). The authorative version is soon to be published in IEEE T-ASE.

## Maintainer

- [Dale McConachie](https://robotics.umich.edu/profile/dale-mcconachie/) <<dmcconac@umich.edu>>, [Autonomous Robotic Manipulation Lab](http://arm.eecs.umich.edu/), University of Michigan

*Note:* ***Please use the GitHub issues*** *for questions and problems regarding the mab_ms package and its components.* ***Do not write emails.***

## Dependencies and Install Details
* This code has been tested on [Ubuntu 16.04](https://www.ubuntu.com/download/desktop) with [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* Each folder is a catkin pacakge, as such this can be added directly to an existing catkin workspace, or used in a new workspace. There are no other ROS packages needed beyond those installed as part of the ros-kinetic-desktop-full package.
* Requires some common libraries such as Eigen. If you find one that is not installed on Ubuntu 16.04 by default, please open and ticket and we will update this list.
* [Gurobi](https://www.gurobi.com)  
  Get academic license  
  Download and [follow installation instructions for 7.0.X](http://www.gurobi.com/documentation/7.0/quickstart_linux/software_installation_guid.html#section:Installation) (extract to /opt; add some lines to .bashrc; run `grbgetkey`)
  Switch to using the g++5.2 version: `cd ${GUROBI_HOME}/lib ` `ln -sf libgurobi_g++5.2.a libgurobi_c++.a`
* mencoder (for generating videos from the simulated experiments): `sudo apt install mencoder`
* `imToMov.sh` needs to be added to the system path somewhere (used in conjuction with mencoder for generating videos). E.g.: `sudo cp imToMov.sh /usr/local/bin`

## Running an experiment
Each experiment uses the same base launch file, and specifies an extra launch file containing parameters specific to that experiment:
* `roslaunch smmap generic_experiment.launch task_type:=rope_cylinder --screen`
* `roslaunch smmap generic_experiment.launch task_type:=cloth_table --screen`
* `roslaunch smmap generic_experiment.launch task_type:=cloth_wafr --screen`

## Logging data
By default, the launch files are setup to log data to `./logs/<task_type>/default`. This will overwrite any data that exists in this folder automatically without prompting. Similarly when screenshots are enabled (see below), the folder `/tmp/smmap_screenshots` will be cleared and recreated every trial.

## Arguments to the outer launch file
`generic_experiment.launch` can take many parameters as arguments, and set these on the ROS parameter server to change how an experiment is run. The [smmap/scripts](https://github.com/UM-ARM-Lab/mab_ms/tree/master/smmap/scripts) folder contains many examples of running tasks in batch mode with varying parameters.

### Usage
Basic pattern:
```
roslaunch smmap generic_experiment.launch task_type:=<rope_cylinder/cloth_table/cloth_wafr> [options:=value]
```

Task and logging arguments:
```
task_type:=<string>
    default: None - needs to be specified
    options: rope_cylinder, cloth_table, cloth_wafr
    info: Controls which experiment to run; a matching "task_type_params.launch" file must exist

logging_enabled:=<boolean>
    default: true
    options: true, false
    info: Controls if data is logged to file while running an experiment

test_id:=<string>
    default: default
    options: Any string that the system can convert to a pth
    info: Controls the subfolder that logging data is saved to. Ie. logs/<task_type>/<test_id>/data_is_saved_here
```

Simulator arguments:
```
launch_simulator:=<boolean>
    default: true
    options: true, false
    info: Controls if the launch file starts the simulator directly or not. Used for debugging purposes in combination with an IDE.

feedback_covariance:=<double>
    default: 0.0
    options: [0, inf]
    info: Controls how much noise the deformable object sensor in the simulator has

start_bullet_viewer:=<boolean>
    default: true
    options: true, false
    info: Controls if the viewer for the simulator is started. Used for batch processing of multiple trials and running on headless machines.

screenshots_enabled:=<boolean>
    default: false
    options: true, false
    info: Requires start_bullet_viewer:=true. Controls if screenshots of the trial are made, and video created at the end.
```

Model usage arguments:
```
multi_model:=<boolean>
    default: false
    options: true, false
    info: Controls if we are using a single model, or a collection of 60 models to manipulate the deformable object

calculate_regret:=<boolean>
    default: false
    options: true, false
    info: Requires multi_model:=true. Controls if we are collecting the data needed to calculate regret plots.

deformability_override:=<boolean>
    default: false
    options: true, false
    info: Requires multi_model:=false use_adaptive_model:=false. Controls if we are going to override the default value used for a diminishing ridigidity model as set in code.

translational_deformability:=<double>
    default: None
    options: [0, inf]
    info: Requires multi_model:=false deformability_override:=true use_adaptive_model:=false. Sets K_trans for a single trial.

rotational_deformability:=<double>
    default: None
    options: [0, inf]
    info: Requires multi_model:=false deformability_override:=true use_adaptive_model:=false. Sets K_rot for a single trial.

use_adaptive_model:=<boolean>
    default: false
    options: true, false
    info: Requires multi_model:=false. Controls if a trial is using a diminishing rigidity model, or an adaptive model.

adaptive_model_learning_rate:=<double>
    default: 0.000001
    options: (0, 1]
    info: Requires multi_model:=false use_adaptive_model:=true. Controls the learning rate for an adaptive model for a single trial.
```

Main control loop and controller settings:
```
launch_planner:=<boolean>
    default: true
    options: true, false
    info: Controls if the planner (main control loop) is started by this launch file.  Used for debugging purposes in combination with an IDE.

optimization_enabled:=<boolean>
    default: true
    options: true, false
    info: Legacy paramter. Can be set to false to change from an Gurobi optimization based controller to pseudo-inverse based controller.

use_random_seed:=<boolean>
    default: false
    options: true, false
    info: Used to get disable deterministic behaviour, enabling statistical trials.

static_seed_override:=<boolean>
    default: false
    options: true, false
    info: Requires use_random_seed:=false. Used to rerun a previous trial to generate the exact same data as was generated previously. Used for debugging purposes.

static_seed:="hex string"
    default = a8710913d2b5df6c
    options: Any hex string that is convertable into a size_t.
    info: Requries use_random_seed:=false static_seed_override:=true. Used for debugging purposes.
```

Multi-armed bandit parameters. The following are meaningful only if `multi_model:=true`:
```
bandit_algorithm:=<string>
    default: KFMANB
    options: UCB, KFMANB, KFMANDB
    info: Controls which MAB method is used to select which model to use at the current timestep to manipulate the object.

kalman_parameters_override:=<boolean>
    default: false
    options: true, false
    info: Controls if we are overriding the default parameters for KFMANDB.

process_noise_factor:=<double>
    default: 0.1
    options: (0, inf)
    info: Requires kalman_parameters_override:=true.

observation_noise_factor:=<double>
    default: 0.01
    options: (0, inf)
    info: Requires kalman_parameters_override:=true.

correlation_strength_factor_override:=<boolean>
    default: false
    options: true, false
    info: Requires kalman_parameters_override:=true.

correlation_strength_factor:=<double>
    default: 0.9
    options: [0, 1]
    info: Requires kalman_parameters_override:=true correlation_strength_factor_override:=true.
```

## F.A.Q.

#### I get the following error `[generic_experiment.launch] is neither a launch file in package [smmap] nor is [smmap] a launch file name`, what is wrong?
Double check that you have built and sourced your catkin workspace correctly.

#### I get the following error and then the simulator closes!
```
================================================================================
REQUIRED process [deform_simulator_node-2] has died!
process has finished cleanly
log file: ~/.ros/log/30ea73d4-3465-11e8-a8bb-0cc47ac81363/deform_simulator_node-2*.log
Initiating shutdown!
================================================================================
```

Check if this message is preceeded by `terminate_simulation_topic not set! Using default of terminate_simulation`. If so, this is normal shutdown behaviour. We have setup the launch files to automatically close everything at the end of a trial to enable batch trials. If not, check to see what happened immediately prior to everything shutting down.

#### The simulator closes almost immedately, and Eigen assertions are failing!

Is the message similar to this?
```
[/smmap_planner_node ros.smmap.planner]: ------------------------------------------------------------------------------------
[/smmap_planner_node ros.smmap.task]: Determining desired direction
[/smmap_planner_node ros.smmap.direct_coverage_task]: Finding 'best' object delta
[/smmap_planner_node ros.smmap.target_point_task]: Found best delta in 0.00554037 seconds
Error code = 10009
No Gurobi license found (user dmcconac, host thor, hostid 7ac81363)
smmap_test_node_KFMANB: /usr/include/eigen3/Eigen/src/Core/Block.h:148: Eigen::Block<XprType, BlockRows, BlockCols, InnerPanel>::Block(XprType&, Eigen::Index, Eigen::Index, Eigen::Index, Eigen::Index) [with XprType = Eigen::Matrix<double, -1, 1>; int BlockRows = 6; int BlockCols = 1; bool InnerPanel = false; Eigen::Index = long int]: Assertion `startRow >= 0 && blockRows >= 0 && startRow <= xpr.rows() - blockRows && startCol >= 0 && blockCols >= 0 && startCol <= xpr.cols() - blockCols' failed.
```
Check your [Gurobi licence](http://www.gurobi.com/documentation/7.0/quickstart_linux/software_installation_guid.html#section:Installation)
