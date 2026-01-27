# Virtual Stage ROS Software Setup <!-- omit in toc -->

Please go through the following steps to set up a computer with the Quanser Virtual Development Environment.

## Description <!-- omit in toc -->

This document describes:

- [System Requirements](#system-requirements)
- [Downloading the ROS Competition Resources](#downloading-the-ros-competition-resources)
- [Setting Up the Isaac-ROS (Development) Container](#setting-up-the-isaac-ros-development-container)
- [How to Run the ROS2 Humble Nodes with the Isaac-ROS Container](#how-to-run-the-ros2-humble-nodes-with-the-isaac-ros-container)
- [Running Nav2](#running-nav2)

By the end of this document, you will have a QCar that is setup to run Nav2 using the Isaac-ROS (Development) container in the below software architecture:

![Physical Software ROS Setup](../Pictures/software_architecture_physical_stage.png)

## System Requirements

`Hardware Requirements:`

- QCar 2

`Software Warning:`

If you plan on changing any native packages on the QCar 2, consult the Software sections at the bottom of the following page: [QCar 2 QUARC Documentation](https://docs.quanser.com/quarc/documentation/qcar2.html)

`Docker Check:`

Docker should be installed by default on the QCar 2. Please check that it is by running the following command:

```bash
docker run hello-world
```

Make sure your QCar 2 is connected to the internet. You should see a `Hello from Docker` somewhere in the output. If this does not work, please raise an issue on the Github.

To remove this container use the following commands:

```bash
docker ps -a
docker rm <CONTAINER ID of hello-world>
```

## Downloading the ROS Competition Resources

The below instructions are inteded to be run on the QCar:

1. Download the [setup_qcar2.py](https://github.com/quanser/student-competition-resources-ros/blob/main/Virtual_ROS_Resources/env_setup/setup_qcar2.py) file

    ![manual download of setup_qcar2.py](../pictures/github_manually_downloading_setup_qcar2.png)

2. Run the `setup_qcar2.py` file from the downloads folder

After running the file, the QCar should have the following folder structure:

``` bash
/home/$USER/Documents/ACC_Development/ 
                        L Development/
                        L isaac_ros_common/
                        L docker/
                        L dev/
                        L backup/
```

## Setting Up the Isaac-ROS (Development) Container

For developing your ROS solution Isaac-ROS container is utilized. This container will communicate with the ROS-Humble nodes running natively on the QCar. Please follow the below steps to set the container up correctly:

Note: Isaac ROS is already installed on the QCar 2. You will only need to install the Nvidia Container Toolkit.

1. To get started please install [Nvidia-Container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#installing-the-nvidia-container-toolkit) using their instructions

    **_NOTE:_**  Make sure you install the [Debian-based installation for the toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#with-apt-ubuntu-debian). And, do not set up the experimental packages repo.

    During the installation of the `nvidia-container-toolkit` you may receive an error that the `nvidia-container-toolkit-base` package is missing. Please install it manually using the following command:

    ```bash
    sudo apt install nvidia-container-toolkit-base
    ```

2. [Configure Your Docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#configuring-docker). You can ignore the Rootless Mode instructions.

    **_NOTE:_**  You may need to add your local user to the local Docker Group. Please restart your machine once your user has been added.

## How to Run the ROS2 Humble Nodes with the Isaac-ROS Container

Follow the below steps to run the ROS2 Humble Nodes natively on the QCar 2 and have the Isaac-ROS container running RViz to visualize some topics.

1. Open a terminal and source ROS2 Humble in the native OS:

    ```bash
    source /opt/ros/humble/setup.bash
    ```

2. Transfer the ROS nodes and interfaces into the ros2 directory:

    ```bash
    cp -r -u /home/$USER/Documents/ACC_Development/Development/ros2/src /home/$USER/ros2
    ```

3. Make sure Python 3.8 is being used to build the nodes:

    ```bash
    PYTHONPATH=$PYTHONPATH:/usr/local/lib/python3.8/dist-packages
    ```

4. Navigate to the ros2 folder and build the nodes:

    ```bash
    cd /home/$USER/ros2
    colcon build
    ```

    There may be some warnings, but these can be ignored.

5. Copy the compiled ROS nodes and interfaces to the container so that the Isaac-ros container has access to the messages the QCar nodes are using:

    ```bash
    cp -r ~/ros2/install/ ~/Documents/ACC_Development/dev/
    ```

6. Source the recently compiled ROS packages:

    ```bash
    source install/setup.bash
    ```

7. Run the nodes using the following command:

    ```bash
    ros2 launch qcar2_nodes qcar2_launch.py
    ```

__________

**<-----From this point forward you will be operating in the Isaac-ROS Container---->**

_____________

8. Open an Isaac-ROS container in a NEW terminal using the following commands:

    ```bash
    cd /home/$USER/Documents/ACC_Development/isaac_ros_common
    ./scripts/run_dev.sh  /home/$USER/Documents/ACC_Development/dev
    ```

    Note: To open additional terminals, run the above 2 commands in a new terminal.

9. Source ROS

    ```bash
    source install/setup.bash
    ```

10. Open RViz2:

    ```bash
    rviz2
    ```

11. Add 'By Topic' relevant topics such as `/laserscan`. For `/laserscan`, switch the Fixed frame in Global options to `base_scan` by typing it in.

## Running Nav2