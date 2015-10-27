# ros-variant

## Synopsis

Topic tools for treating ROS messages as type-erased variants.

**Author(s):** Ralf Kaestner

**Maintainer:** Ralf Kaestner <ralf.kaestner@gmail.com>

**Licsense:** GNU Lesser General Public License (LGPL)

**Operating system(s):** Debian-based Linux

**Package PPA:** Not available

## Description

This project provides a partially templated C++ API for treating ROS messages
as type-erased variants. Similar to the Python API of ROS, it thus renders ROS
messages and their members accessible without the requirement for including any
message headers at compile time. The goal of this functionality is to motivate
developers to contribute more C++ packages which are intended to inspect and
operate on any message type.

This project further provides a test package which practically demonstrates
API usage by example.

## Installation

### Installing from packages (recommended for Ubuntu LTS users)

The maintainers of this project do not yet provide binary packages.

### Building from source

This project may be built from source using catkin.

Here, we assume you intend to build the project for the ROS distribution
named `ROS_DISTRO`.

#### Installing build dependencies

The build dependencies of this project are available from the standard
package repositories of recent Ubuntu and ROS releases. To install them,
simply use the command

```
sudo apt-get install ros-ROS_DISTRO-roscpp, ros-ROS_DISTRO-std-msgs

```

#### Building with catkin (recommended for ROS developers)

Assuming that you have cloned the project sources into `PROJECT_DIR`, you
may attempt to build this project using catkin as follows:

* Create the directory structure for your catkin workspace by issuing

  ```
  mkdir -p CATKIN_WS_DIR/src
  ```

* Assuming that your ROS environment has been set up properly, initialize the
  catkin workspace using the command

  ```
  catkin_init_workspace CATKIN_WS_DIR/src
  ```

* Individually link the directories containing the project's package source
  trees into your catkin workspace source directory through

  ```
  ln -sf PROJECT_DIR/roscpp_nodewrap* CATKIN_WS_DIR/src
  ```

* Switch into the catkin workspace directory by 

  ```
  cd CATKIN_WS_DIR
  ```

* In the catkin workspace directory, run 

  ```
  catkin_make
  ```

  to start the build

## API documentation

This project does not yet provide any API documentation.

## Feature requests and bug reports

If you would like to propose a feature for this project, please consider
contributing or send a feature request to the project authors. Bugs may be
reported through the project's issue page.

## Further reading

For additional information of the Robot Operating System (ROS), please refer
to the official [ROS documentation](http://wiki.ros.org).
