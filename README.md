# variant

## Synopsis

Topic tools for treating ROS messages as type-erased variants.

**Author(s):** Ralf Kaestner

**Maintainer:** Ralf Kaestner <ralf.kaestner@gmail.com>

**Licsense:** GNU Lesser General Public License (LGPL)

**Operating system(s):** Debian-based Linux, Mac OS X

**Package PPA:** Not available

## Description

This project provides a partially templated C++ API for treating ROS messages
as type-erased variants. Similar to the Python API of ROS, it thus renders ROS
messages and their members accessible without the requirement for including any
message headers at compile time. The goal of this functionality is to encourage
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

```shell
sudo apt-get install ros-ROS_DISTRO-roscpp, ros-ROS_DISTRO-std-msgs
```

#### Building with catkin (recommended for ROS developers)

Assuming that you have cloned the project sources into `PROJECT_DIR`, you
may attempt to build this project using catkin as follows:

* Create the directory structure for your catkin workspace by issuing

  ```shell
  mkdir -p CATKIN_WS_DIR/src
  ```

* Assuming that your ROS environment has been set up properly, initialize the
  catkin workspace using the command

  ```shell
  catkin_init_workspace CATKIN_WS_DIR/src
  ```

* Individually link the directories containing the project's package source
  trees into your catkin workspace source directory through

  ```shell
  ln -sf PROJECT_DIR/roscpp_nodewrap* CATKIN_WS_DIR/src
  ```

* Switch into the catkin workspace directory by 

  ```shell
  cd CATKIN_WS_DIR
  ```

* In the catkin workspace directory, run 

  ```shell
  catkin_make
  ```

  to start the build

## API documentation

This project does not yet build its API documentation. Some basic documentation
is however inlined with the interface definitions and can thus be found in the
source headers.

## Feature requests and bug reports

If you would like to propose a feature for this project, please consider
contributing or send a feature request to the project authors. Bugs may be
reported through the project's issue page.

## Further reading

For additional information of the Robot Operating System (ROS), please refer
to the official [ROS documentation](http://wiki.ros.org).

## Build Status

### Devel Job Status

| | Indigo  | Jade | Kinetic |
| --- | --- | --- | --- |
| variant | [![Build Status](http://build.ros.org/buildStatus/icon?job=Idev__variant__ubuntu_trusty_amd64)](http://build.ros.org/job/Idev__variant__ubuntu_trusty_amd64/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Jdev__variant__ubuntu_trusty_amd64)](http://build.ros.org/job/Jdev__variant__ubuntu_trusty_amd64/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kdev__variant__ubuntu_xenial_amd64)](http://build.ros.org/job/Kdev__variant__ubuntu_xenial_amd64/) |

### Release Job Status

| | Indigo | Jade | Kinetic |
| --- | --- | --- | --- |
| variant | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ibin_uT64__variant__ubuntu_trusty_amd64__binary)](http://build.ros.org/job/Ibin_uT64__variant__ubuntu_trusty_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Jbin_uT64__variant__ubuntu_trusty_amd64__binary)](http://build.ros.org/job/Jbin_uT64__variant__ubuntu_trusty_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__variant__ubuntu_xenial_amd64__binary)](http://build.ros.org/job/Kbin_uX64__variant__ubuntu_xenial_amd64__binary/) |
| variant_msgs | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ibin_uT64__variant_msgs__ubuntu_trusty_amd64__binary)](http://build.ros.org/job/Ibin_uT64__variant_msgs__ubuntu_trusty_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Jbin_uT64__variant_msgs__ubuntu_trusty_amd64__binary)](http://build.ros.org/job/Jbin_uT64__variant_msgs__ubuntu_trusty_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__variant_msgs__ubuntu_xenial_amd64__binary)](http://build.ros.org/job/Kbin_uX64__variant_msgs__ubuntu_xenial_amd64__binary/) |
| variant_topic_tools | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ibin_uT64__variant_topic_tools__ubuntu_trusty_amd64__binary)](http://build.ros.org/job/Ibin_uT64__variant_topic_tools__ubuntu_trusty_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Jbin_uT64__variant_topic_tools__ubuntu_trusty_amd64__binary)](http://build.ros.org/job/Jbin_uT64__variant_topic_tools__ubuntu_trusty_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__variant_topic_tools__ubuntu_xenial_amd64__binary)](http://build.ros.org/job/Kbin_uX64__variant_topic_tools__ubuntu_xenial_amd64__binary/) |
