# thesis-2024-Podliesnova
## Contents: 
* [1. Introduction](#1-introduction)
* [2. Pre-processing and detection of blink and ErrP events](#2-pre-processing-and-detection-of-blink-and-errp-events)
* [3. Applying custom ground for Gazebo simulation](#3-applying-custom-ground-for-gazebo-simulation)
* [4. Running the simulation](#4-running-the-simulation)
## 1. Introduction
The rapidly growing field of Brain-Computer Interfaces (BCIs) provides an innovative method for human-robot interaction, enabling machines to be controlled directly via brain signals. This repository supports study that investigates the use of BCI technology to monitor the operator’s brain activity in real-time and detect robot malfunctions through the identification of Error-Related Potentials (ErrPs).

It contains:
1. Signal pre-processing and Machine Learning codes and models for both blinking and ErrP events.
2. Created custom texture for Gazebo simulation.
3. ROS package with real-time system for acquiring, processing, and detecting blink/ErrP events together with working simulation of real-world application.

## 2. Pre-processing and detection of blink and ErrP events
Jupyter notebook files `blink_events/blinking_dataset_pipeline.ipynb` and `errp_events/errp.ipynb` were used for processing and training ML models from open-source datasets. Corresponding best performance models can be found in `models/` folder.
## 3. Applying custom ground for Gazebo simulation
1. Place `texture_ground_model/line/materials/textures/ground7.png` to `/usr/share/gazebo-(version_number)/media/materials/textures/` using `sudo`.
2. Git clone `robotont_gazebo` [repository](https://github.com/robotont/robotont_gazebo):
```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/robotont/robotont_gazebo
$ cd ~/catkin_ws
$ catkin build
$ source devel/setup.bash
```
3. Paste the `texture_ground_model/line` folder inside the `robotont_gazebo/models`.
4. Launch simulation `world_minimaze.launch`:
```bash
$ roslaunch robotont_gazebo world_minimaze.launch
```
5. After the Gazebo GUI loaded, go to `Insert` tab, `models`, choose the `line` model and place it as a ground of the maze.
6. In order to avoid rendering issues, remove the default ground plane of the simulation.
## 4. Running the simulation
1. Place `bci_sim` package in your `catkin_ws/src` folder and build the workspace.
2. Put on your head OpenBCI device and insert USB Bluetooth Dongle into the PC.
3. Turn on OpenBCI device.
4. Run `bci_errp` (or `bci_blinking` to test the pipeline) node:
```bash
$ cd ~/catkin_ws
$ rosrun bci_sim bci_errp.py
```
5. Wait some time for device to configure
6. Run movement node:
```bash
$ cd ~/catkin_ws
$ rosrun bci_sim move_on_line_random_stop.py
```
6. Run evaluation node (if needed):
```bash
$ cd ~/catkin_ws
$ rosrun bci_sim eval.py
```
