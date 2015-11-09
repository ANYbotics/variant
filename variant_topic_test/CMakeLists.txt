cmake_minimum_required(VERSION 2.8.3)
project(variant_topic_test)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
# set(CMAKE_BUILD_TYPE RelWithDebInfo)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    cmake_modules
    std_msgs
    geometry_msgs
    roscpp
    variant_msgs
    variant_topic_tools
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
  LIBRARIES variant_topic_test
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    variant_msgs
    variant_topic_tools
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

###########
## Tests ##
###########

catkin_add_gtest(variant_topic_test
  test/variant_topic_test
  test/DataTypeTest.cpp
  test/DataTypeRegistryTest.cpp
  test/MD5SumTest.cpp
  test/MessageDefinitionTest.cpp
  test/MessageDefinitionParserTest.cpp
  test/MessageFieldCollectionTest.cpp
  test/MessageTest.cpp
  test/MessageTypeParserTest.cpp
  test/MessageTypeTest.cpp
  test/PointerTest.cpp
  test/SerializerTest.cpp
  test/VariantTest.cpp
)

target_link_libraries(
  variant_topic_test
  ${catkin_LIBRARIES}
)
