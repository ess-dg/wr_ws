# Welder Robot
ROS-based welder robot.

# Prereqs
- Ubuntu 18.04
- ROS Melodic

Also install
> libunwind-dev ros-melodic-image-transfer ros-melodic-cv-bridge

Run catkin\_make from the root dir to build the project.

USB-FS should be increased to 16 MB to allow for more RAM for buffering (of images), which can be done temporarily like `sudo sh -c "echo 1000 > /sys/module/usbcore/parameters/usbfs_memory_mb"`.
