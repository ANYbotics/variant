cmake_minimum_required(VERSION 3.16.3)
project(variant_msgs)

if("$ENV{ROS_VERSION}" STREQUAL "1")
  # gersemi: off


# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
# set(CMAKE_BUILD_TYPE RelWithDebInfo)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    std_msgs
)

####################################
## Generate messages and services ##
####################################
## Add message files to be generated
add_message_files(
  DIRECTORY msg
  FILES
    Test.msg
    Variant.msg
    VariantHeader.msg
    VariantType.msg
)

## Generate services
generate_messages(
  DEPENDENCIES std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
)


  # gersemi: on
else() # ROS version 2
  ###############
  ## Configure ##
  ###############

  # Set the build type.  Options are:
  #  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
  #  Debug          : w/ debug symbols, w/o optimization
  #  Release        : w/o debug symbols, w/ optimization
  #  RelWithDebInfo : w/ debug symbols, w/ optimization
  #  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
  # set(CMAKE_BUILD_TYPE RelWithDebInfo)

  ## Find catkin macros and libraries

  set(
    PACKAGE_DEPENDENCIES
    builtin_interfaces
    std_msgs
  )

  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)

  foreach(PKG ${PACKAGE_DEPENDENCIES})
    find_package(${PKG} REQUIRED)
  endforeach()

  rosidl_generate_interfaces(
    ${PROJECT_NAME}
      "msg/Test.msg"
      "msg/Variant.msg"
      "msg/VariantHeader.msg"
      "msg/VariantType.msg"
    DEPENDENCIES
      ${PACKAGE_DEPENDENCIES}
  )
  ####################################
  ## Generate messages and services ##
  ####################################
  ## Add message files to be generated

  ## Generate services

  ###################################
  ## catkin specific configuration ##
  ###################################
  ## The catkin_package macro generates cmake config files for your package
  ## Declare things to be passed to dependent projects
  ## INCLUDE_DIRS: uncomment this if you package contains header files
  ## LIBRARIES: libraries you create in this project that dependent projects also need
  ## CATKIN_DEPENDS: catkin_packages dependent projects also need
  ## DEPENDS: system dependencies of this project that dependent projects also need
  ament_export_dependencies(rosidl_default_generators)
  ament_export_dependencies(${PACKAGE_DEPENDENCIES})

  ament_package()
endif()
