This is a ROS 2 component that can be used to add a custom flight mode to a PX4 autopilot which will cause it fly in the shape of the letter R. It relies on [Auterion's PX4 ROS2 interface library](https://github.com/Auterion/px4-ros2-interface-lib) which has a number of other examples worth checking out.

## Requirements 
- Ubuntu (22.04)
- PX4 and ROS 2 installations as detailed by https://docs.px4.io/main/en/ros2/user_guide.html 
    - This involves setting up the PX4 toolchain, ROS 2, and the Micro XRCE-DDS Agent 
    - Tested on PX4 v1.15.0 and v1.15.4, ROS 2 Humble
    - If this is a first time setup, try at least one of the examples at the end of the installation guide above before moving on to confirm you have everything set up properly.
- QGroundControl
    - The most recent available releases (daily and V4.4.3 as of writing) do not support dynamically adding external flight modes that are visible in the flight modes drop down menu in QGC. However, building the Stable_V4.4 branch manually does support this. It doesn't matter for this example, however, since the flight mode will activate automatically.

## Setup
1. Setup and build a new ROS workspace
    ```bash
    # Make a new folder for the workspace
    mkdir -p custom_flightmode/src
    cd custom_flightmode/src

    # Clone components
    git clone --branch release/1.15 https://github.com/PX4/px4_msgs.git
    git clone --recursive https://github.com/Auterion/px4-ros2-interface-lib.git
    git clone <this repo>

    # Build the workspace and setup the shell to run ros commands later
    cd ..
    source /opt/ros/humble/setup.bash
    colcon build
    source install/setup.bash
    ```
2. **In a new terminal**, start MicroXRCEAgent
    ```bash
    MicroXRCEAgent udp4 -p 8888
    ```
3. **In another new terminal**, start a Gazebo PX4 simulation
    ```bash
    cd ~/PX4-Autopilot # Change this depending on where you installed PX4
    
    # Ignore these 2 lines if you are already on the correct version of PX4
    git checkout v1.15.4
    git submodule update --init --recursive

    make px4_sitl gz_x500
    ```
4. Start QGroundControl. It should connect to the drone in the simulator.
5. **Return to the first terminal** and run the following. If you closed the terminal, you'll need to run the 2 `source` commands from the first step again beforehand.
    ```bash
    ros2 run draw_flight_mode draw_r
    ```
6. The drone should automatically arm, takeoff, draw the letter R, and then land. QGC may display a weird flight mode value; ignore this.
    - You can check that the custom flight mode is recognized by running `commander status` in the simulator shell. You should see an external flight mode named "Draw R".