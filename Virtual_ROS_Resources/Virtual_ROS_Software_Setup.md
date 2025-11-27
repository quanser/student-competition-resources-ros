# Virtual Stage ROS Software Setup <!-- omit in toc -->

Please go through the following steps to set up a computer with the Quanser Virtual Development Environment.

## Description <!-- omit in toc -->

This document describes:

- [System Requirements](#system-requirements)
- [How to Set Up the Quanser Virtual Environment Docker Container](#how-to-set-up-the-quanser-virtual-environment-docker-container)
- [Set Up the Development Docker Container](#set-up-the-development-docker-container)
- [How to Run the ROS2 Humble Nodes](#how-to-run-the-ros2-humble-nodes)

By the end of this document, you will have a Quanser Virtual Environment Docker Container that can be used to spawn the QCar 2 in QLabs. You will have a Development Docker Container where you will develop your code and run the ROS2 nodes. And, you will have QLabs installed. The configuration will look like this:

![QLabsDevelopmentVennDiagram](../Pictures/DevelopmentVennDiagram.png)

## System Requirements

Linux System base requirements:

- Ubuntu 24.04 LTS
- Nvidia Based GPU

Please follow this guide to correctly setup Docker engine on your ground station PC: \
[Docker Installation Instructions](https://docs.docker.com/engine/install/ubuntu/)

Download and set up the Docker Container Resources:

1. Download `setup_linux.py` manually from here: [setup_linux.py](/Virtual_ROS_Resources/env_setup/setup_linux.py)

    ![manually downloading file](../Pictures/github_manually_downloading_file.png)

2. Run `setup_linux.py`

Your system should look like this after running `setup_linux.py`:

``` bash
/home/$USER/Documents/ACC_Development/ 
                        L backup/
                        L Development/
                        L docker/
                        L isaac_ros_common/
```

## How to Set Up the Quanser Virtual Environment Docker Container

The purpose of this docker container is to ensure safe setup of the Quanser Interactive Labs virtual environment. The below instructions go over how to set this Docker container up.

1. Download the latest debian packages (only need to do once):

    ``` bash
    wget --no-cache https://repo.quanser.com/debian/release/config/configure_repo.sh
    chmod u+x configure_repo.sh
    ./configure_repo.sh
    rm -f ./configure_repo.sh
    sudo apt update 
    ```

2. Install QLabs, Quanser Python APIs, and QUARC runtime:

    ```bash
    sudo apt-get install qlabs-unreal python3-quanser-apis  quarc-runtime
    ```

3. Register an account in the [Quanser Academic Portal](https://portal.quanser.com/Accounts/Register) to obtain access for Quanser Interactive Labs.

4. To setup Quanser's custom python docker navigate to the folder:

    ```bash
    cd /home/$USER/Documents/ACC_Development/docker/development_docker/quanser_dev_docker_files
    ```

    And run the following command:

    ```bash
    sudo docker run --rm -it --network host --name virtual-qcar2 quanser/virtual-qcar2 bash
    ```

    This docker container will include the following setup scripts:

    ```bash
    /home/qcar2_scripts/python 
                            L Base_Scenarios_Python/
    ```

5. **To open additional terminals attached to the docker container**, run the following command in a new terminal:

    ```bash
    docker exec -it virtual-qcar2 bash
    ```

**To pull the most recent version of the Docker container (after you go through the below steps first) from Dockerhub use the following command:**

```bash
docker pull quanser/virtual-qcar2:latest
```

## Set Up the Development Docker Container

For software development we will leverage the isaac_ros docker container. This container can be used for:

- Python only work (Not using ROS)
- ROS focused solutions (Using either python/C++ to write ROS nodes)

The below instructions show you how to set up the **Development Docker Container**.

1. To get started please install [Nvidia-Container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#installing-the-nvidia-container-toolkit)

    **_NOTE:_**  If you're not sure what method to use, scroll to the top of the page and follow the With apt: Ubuntu, Debian' section. You will not need to configure the repository to use experimental packages. Then make sure you [configure docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#configuring-docker).

2. Navigate to the following directory:

    ```bash
    cd /home/$USER/Documents/ACC_Development/isaac_ros_common
    ```

3. To start the container use the command

    ```bash
    ./scripts/run_dev.sh  /home/$USER/Documents/ACC_Development/Development
    ```

    **_NOTE:_**  You may need to add your local user to the local Docker Group. Please restart your machine once your user has been added.

4. **To open additional terminals attached to the Development docker container**, open a new terminal and run Steps 2 & 3 again.

## How to Run the ROS2 Humble Nodes

Once you are ready to start developing, follow these steps to start the virtual environment:

1. Natively in Ubuntu, open the QLabs application and navigate to the SDCS then the Open Plane.

2. If you do not have a Quanser Virtual Environment Docker container open, follow the above sections to open one.

3. Using the Quanser Virtual Environment Docker container, navigate to the following directory:

    ```bash
    cd /home/qcar2_scripts/python
    ```

4. Run the following Python script to spawn the competition map into QLabs:

    ```bash
    python3 Base_Scenarios_Python/Setup_Competition_Map.py
    ```

Once everything has run to completion, the QLabs world should look like the following:

![QLabs after running Setup_Competition_Map.py](../Pictures/HowToStart.png)

1. If you do not have a Development Docker (Isaac-ROS) container open, follow the above section to open one. The following commands should be run in the Development container.

2. Compile the QCar2 ros nodes using:

    ```bash
    colcon build
    ```

3. Source the QCar2 packages using:

    ```bash
    . install/setup.bash
    ```

4. Launch the nodes for the QCar using the launch file configured for the virtual QCar:

    ```bash
    ros2 launch qcar2_nodes qcar2_virtual_launch.py
    ```

**IMPORTANT:** For tips and guides on how to develop in this container, visit the [Virtual ROS Devlopment Guide](/Virtual_ROS_Resources/Virtual_ROS_Development_Guide.md).
