# ROS Nodes

Put this repository in your catkin workspace and install to use the ROS nodes.

### Braid Emulator

Requirements:
* PyNumDiff: `pip install pynumdiff`
* Ros flydra: https://github.com/strawlab/ros_flydra

Emulating braid. To test things out, you can use the emulator. By default the emulator will randomly generate some trajectories that are born and die and move with some sinusoidal behavior. rosrun braid_trigger braid_emulator.py. You can customize your trajectories if you like, look at braid_emulator.py's main script for some hints.

### Braid Trigger in Volume

Runing the trigger. An example volumetric trigger is given in braid_trigger_in_volume.py. It requires a configuation file, and example is given in configurations/. To run the trigger, run `rosrun braid_trigger braid_trigger_in_volume.py --config=PATH_TO_CONFIGURATION`.

### Braid Realtime Plotter

# Analysis Code

Run `python ./setup.py install` to install the analysis package.