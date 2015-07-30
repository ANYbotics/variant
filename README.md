# ros-topic-variant

## Synopsis

C++ implementation of ROS node(let) wrappers.

**Author(s):** Ralf Kaestner

**Maintainer:** Ralf Kaestner <ralf.kaestner@gmail.com>

**Licsense:** GNU Lesser General Public License (LGPL)

**Operating system(s):** Debian-based Linux

**Package PPA:** Not available

## Description

This project provides templated C++ class wrappers for implementing the core
functionalities of ROS nodes and nodelets based on the object adapter design
pattern. The main features available to the node(let) developer are:

* Flexible instantation of the node(let) implementation as ROS node or
  nodelet, allowing for the following runtime use cases:
  * Dedicated ROS node (single node implementation per process)

  * ROS "supernode" (multiple node implementations per process)

  * ROS nodelets (plugin-based loading and unloading of node implementations
    by the nodelet manager)
    
* Signal-based cleanup handlers

* Configuration server advertising a node(let)'s parameters as ROS services

* Callback handlers for parameter changes (either using rosparam or ROS
  services)

This project further provides a tutorial package which practically demonstrates
the different use cases by example.

## Installation

### Installing from packages (recommended for Ubuntu LTS users)

The maintainers of this project do not yet provide binary packages.

### Building from source

This project maintains two parallel build system branches and may either
be built using catkin or the CMake build system with an open-source macro
extension called ReMake.

Here, we assume you intend to build the project for the ROS distribution
named `ROS_DISTRO`.

#### Installing build dependencies

The build dependencies of this project are available from the standard
package repositories of recent Ubuntu and ROS releases. To install them,
simply use the command

```
sudo apt-get install ros-ROS_DISTRO-roscpp, ros-ROS_DISTRO-nodelet ros-ROS_DISTRO-rospy, ros-ROS_DISTRO-message-generation, ros-ROS_DISTRO-std-msgs

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

#### Building with ReMake

##### Preparing the build system

If you already have installed ReMake on your build system, you may
skip this step. Otherwise, before attempting to build this project the
traditional CMake way, you must install ReMake following
[these instructions](https://github.com/kralf/remake).

##### Building with CMake

Once ReMake is available on your build system, you may attempt to build this
project the CMake way. Assuming that you have cloned the project sources into
`PROJECT_DIR`, a typical out-of-source build might look like this:

* Create a build directory using 

  ```
  mkdir -p PROJECT_DIR/build
  ```

* Switch into the build directoy by 

  ```
  cd PROJECT_DIR/build
  ```

* In the build directory, run 

  ```
  cmake -DROS_DISTRIBUTION=ROS_DISTRO PROJECT_DIR
  ```

  to configure the build

* If you want to inspect or modify the build configuration, issue 

  ```
  ccmake PROJECT_DIR
  ```

* Build the project using 

  ```
  make
  ```

* If you intend to install the project, call 

  ```
  make packages_install
  ```

  (from packages on Debian-based Linux only) or 

  ```
  make install
  ```

## API documentation

This project generates its API documentation from source. To access it, you
may either inspect the build directory tree after the project has been built
using `make` or install the documentation package through

```
sudo apt-get install roscpp-nodewrap-doc
```

## Feature requests and bug reports

If you would like to propose a feature for this project, please consider
contributing or send a feature request to the project authors. Bugs may be
reported through the project's issue page.

## Further reading

For additional information of the Robot Operating System (ROS), please refer
to the official [ROS documentation](http://wiki.ros.org).
