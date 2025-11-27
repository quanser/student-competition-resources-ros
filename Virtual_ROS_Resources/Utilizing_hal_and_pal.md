# Utilizing `hal` and `pal` in the Development Container <!-- omit in toc -->

Two Python libraries from Quanser's standard resources are included in the Development Container by default, `hal` and `pal`.

`hal` stands for high-level applications library and contains userful functions like EKF's, image processing, and controllers.

`pal` stands for product application library and contains userful QCar functions like creating a QCar class that can read and write information to the QCar, and other classes for interfacing with the lidar and cameras.

There is an [official libraries guide](https://github.com/quanser/Quanser_Academic_Resources/blob/dev-windows/0_libraries/libraries_guide.pdf) for `hal/pal` in the Quanser Academic Resources.

This guide will cover all the information necessary to utilize the `hal` and `pal` libraries in the Development Container.

- [Location of Libraries](#location-of-libraries)
- [Examples using `hal/pal`](#examples-using-halpal)

## Location of Libraries

`hal/pal` are downloaded as part of the ACC_Resources. They are stored within:

```bash
cd /home/$USER/Documents/ACC_Development/docker/0_libraries/python
```

Everytime the Development Container is started, it will pull these libraries into the Development Container here:

```bash
cd /home/Quanser/0_libraries/python
```

There is an environment variable in the development container called `$PYTHONPATH` that points towards 0_libraries.

## Examples using `hal/pal`

For those of you that are using only Python, some example code has been provided in the following folder:

```bash
cd /home/$USER/Documents/ACC_Development/Development/python_dev
```

This code will be copied into the Development Container here:

```bash
cd /workspaces/isaac_ros-dev/python_dev
```

There are basic hardware tests that showcase how to interface with the QCar 2.
