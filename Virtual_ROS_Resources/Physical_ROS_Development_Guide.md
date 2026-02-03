# Physical ROS Development Guide <!-- omit in toc -->

This document will show the architecture of the ROS software on the physical QCar 2 and describes the intended development pipeline for utilizing the Isaac-ROS container and the native ROS Humble installation.

- [QCar2 ROS Software Architecture](#qcar2-ros-software-architecture)
- [How to Transfer the Virtual Isaac-ROS Container to the Physical System](#how-to-transfer-the-virtual-isaac-ros-container-to-the-physical-system)
- [How to create permanent files in the Isaac-ROS Container](#how-to-create-permanent-files-in-the-isaac-ros-container)
- [How to add packages via `apt` that persist in the Isaac-ROS container](#how-to-add-packages-via-apt-that-persist-in-the-isaac-ros-container)
- [How to add Python packages via `pip3` that persist in the Isaac-ROS container](#how-to-add-python-packages-via-pip3-that-persist-in-the-isaac-ros-container)
- [How to Run the Isaac-ROS Container with No Internet Connection](#how-to-run-the-isaac-ros-container-with-no-internet-connection)

## QCar2 ROS Software Architecture

On the physical QCar 2, the native ROS installation will be used to run the QCar 2 nodes, while all the development will take place within the Isaac-ROS container. It is intended that you do not modify the native ROS2 nodes and instead only add/change packages in the Isaac-ROS container. This is to ensure safe development.

![Development Structure](../Pictures/software_architecture_physical_stage.png)

**WARNING:**

Updating packages in the native Ubuntu OS on the QCar 2 can damage the QCar 2 software irrepairably. Before updating any packages consult the Software section of the following webpage: [QCar2 QUARC Documentation](https://docs.quanser.com/quarc/documentation/qcar2.html).

## How to Transfer the Virtual Isaac-ROS Container to the Physical System

Any files that were stored within the `/home/$USER/Documents/ACC_Development/Development` folder in the Virtual Stage should be saved and transferred over to the same folder on the QCar 2. These will still get pulled into the Isaac-ROS Container when the container is launched.

Any Debian or Python packages you installed on the Isaac-ROS container through `Dockerfile.quanser` in the Virtual Stage must be transferred over to the `Dockerfile.student`.

## How to create permanent files in the Isaac-ROS Container

Any files placed within the `/home/$USER/Documents/ACC_Development/Development` folder will be pulled into the container when the Isaac-ROS Container is launched and will persist after the container is closed. Keep all development files within this container.

## How to add packages via `apt` that persist in the Isaac-ROS container

Any packages that get installed via `apt` in the command line of the Isaac-ROS Container do not persist once the container is closed. When you are installing packages in the terminal, make sure to record those packages in `Dockerfile.student`. This Docker file is used to configure the Isaac-ROS container and can be found in the following folder:

```bash
cd /home/$USER/Documents/ACC_Development/docker/development_docker/quanser_dev_docker_files
```

At the bottom of the `Dockerfile.student` add your Debian packages as shown below:

```bash
# Install Debian packages
RUN apt-get update && apt-get install -y \
    python3-pyqtgraph \
    && rm -rf /var/lib/apt/lists/*
```

This example will install pytransform3d and pyqtgraph everytime the container is started. **As you develop in the Development Container**, make sure you record any packages installed via `apt` in the Docker file mentioned above.

**IMPORTANT:** If you run the `setup_qcar2.py` file again, this will create a new ACC_Development folder and record a backup of your previous one. Make sure to copy any changes made to `Dockerfile.student`.

## How to add Python packages via `pip3` that persist in the Isaac-ROS container

**NOTE**: We recommend you use the Debian packages if they are available.

Any packages that get installed via `pip3` in the command line of the Isaac-ROS Container do not persist once the container is closed. When you are installing packages in the terminal, make sure to record those packages in `Dockerfile.student`. This Docker file is used to configure the Isaac-ROS container and can be found in the following folder:
```bash
cd /home/$USER/Documents/ACC_Development/docker/isaac_ros
```

At the bottom of the `Dockerfile.student` add your Python packages as shown below:

```bash
# Install Python Packages if necessary
RUN pip3 install -U \
    pytransform3d \
    pyqtgraph
```

This example will install pytransform3d and pyqtgraph everytime the container is started. **As you develop in the Isaac-ROS Container**, make sure you record any packages installed via `pip3` in the Docker file mentioned above.

**IMPORTANT:** If you run the `setup_qcar2.py` file again, this will create a new ACC_Development folder and record a backup of your previous one. Make sure to copy any changes made to `Dockerfile.student`.

## How to Run the Isaac-ROS Container with No Internet Connection

When you run the Isaac-ROS container with the script `run_dev.sh` this will check if there is an updated version of the Base Isaac-ROS container. This requires an internet connection.

To run the Isaac-ROS container with no internet connection, use the commands:

```bash
cd /home/$USER/Documents/ACC_Development/isaac_ros_common
chmod a+x ./scripts/run_local_dev.sh
./scripts/run_local_dev.sh  /home/$USER/Documents/ACC_Development/dev
```

These commands will pull on the locally built Docker image and not check online for an updated one.

Note: Once you have changed `run_local_dev.sh` to an executable file, you don't need to run the chmod command again.