# Stonefish_BlueROV2 ROS2
Simulaton of Blue Robtics BlueROV2 platform in Stonefish simulator with python plugin to attach the simulator to ArduSub SITL (Software-in-the-loop) in ROS2

Tested on:
- Ubuntu 24.04 with ROS Jazzy.

Requirements:
- ROS2 with colon and DDS backend.
- Stonefish Simulator installed as library.
- Stonefish ROS2 wrapper.
- Ardupilot SITL installation
- OPTIONAL: ROS2 BlueROV2 driver: https://github.com/bvibhav/bluerov2_interface

# ROS2
Refer to installation at https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html

# Stonefish
Refer to documentation at https://github.com/patrykcieslak/stonefish for installation. Checkout following brach for scene files compatibilty before building the library. 

`git checkout v1.3`

# Stonefish ROS
Create a colcon workspace as suggested in ROS2 documentation: https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html

Refer to installation instruction at https://github.com/patrykcieslak/stonefish_ros2. Make sure to checkout follwoing branch before building. 

`git checkout v1.3`

# Ardupilot SITL
Refer to this page for setting up Ardupolit SITL: https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux.

Make sure to install all submodules if things don't work as intended. 

```
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot
cd ardupilot
```

Don't forget to install the required packages as suggested in the documentation and adding to linux profile. 

```
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

Ardupilot should be ready to use at this point. 

## Testing ArduPilot SITL for BlueROV2
Run following commands to test that ArduSub is running porperly. For the first time the system will build the firmware for ArduPilot SITL to use. 

```
cd ardupilot 
sim_vehicle.py -v ArduSub --map 
```

This will put the simulated vehicle in middle of nowhere going in to the ground. JUST AMAZING!!!

cd into this location in `ardupilot` directory `/home/bvibhav/Ardupilot/Tools/autotest/` and edit `locations.txt`. Add this line at the very bottom. 

`PHILL=56.026930,-3.385670,0,0`

Now run the commands again

```
cd ardupilot 
sim_vehicle.py -v ArduSub --map -L PHILL
```

At this point, you should be able to control the vehicle with `QGroundControl` (https://qgroundcontrol.com/downloads/).

## Running Ardupilot SITL with your own simulator
Run this command to start SITL with JSON simulator instead. This will let us connect to our own simulator on the local machine (this can be changed later for different IP).

```
sim_vehicle.py -v ArduSub --model JSON  --map -L PHILL -m --streamrate=-1
```

This will complain about link 1 down as SITL is not getting anything from simulator side. 

# Buidling/Running this Simulation
- Clone this repository to your colcon workspace.
- Build the package using symlink method like this: 

    ```
    colcon build --event-handlers console_direct+ --cmake-args --symlink-install --packages-select stonefish_bluerov2
    ```

- Then launch the simulation: 

    ```
    ros2 launch stonefish_bluerov2 bluerov2_sim.py
    ```

- Finally, open QGroundControl, got to vehicle setup and chnage the vehicle configuation to `Vectored-6DOF`.
- Also enable jostick intput in QGC. 

At this point, you are setup and should be able to control the vehicle using QGC or BlueROV2 driver of your choice. 

You can use a barebones ROS2 driver from here: https://github.com/bvibhav/bluerov2_interface

# Running Rover/Boat Simulation
- Vehicle frame `-v Rover` or `-v ArduSub` is not required if running in firmware type folder. This ideal if you are going to run multiple firmware using same Ardupilot directory. 
    ```
    cd ~/ardupilot/Rover
    sim_vehicle.py --model JSON --map -L PHILL -m --streamrate=-1
    ```

- Run the simulation side of things. 
    ```
    ros2 launch stonefish_bluerov2 blueboat_sim.py
    ```

- Run QGroundControl and for the first time loat the default parameters from here: https://github.com/bluerobotics/Blueos-Parameter-Repository/tree/master/params/ardupilot/ArduRover/4.5/Navigator