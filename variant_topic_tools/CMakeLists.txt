cmake_minimum_required(VERSION 3.5.1)
project(variant_topic_tools)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
# set(CMAKE_BUILD_TYPE RelWithDebInfo)

add_definitions(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    roslib
    variant_msgs
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
  INCLUDE_DIRS include
  LIBRARIES variant_topic_tools
  CATKIN_DEPENDS
    roscpp
    roslib
    variant_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(variant_topic_tools
  src/ArrayDataType.cpp
  src/ArraySerializer.cpp
  src/ArrayVariant.cpp
  src/BuiltinDataType.cpp
  src/BuiltinSerializer.cpp
  src/BuiltinVariant.cpp
  src/CollectionVariant.cpp
  src/DataType.cpp
  src/DataTypeRegistry.cpp
  src/Exceptions.cpp
  src/MD5Sum.cpp
  src/Message.cpp
  src/MessageConstant.cpp
  src/MessageDataType.cpp
  src/MessageDefinition.cpp
  src/MessageDefinitionParser.cpp
  src/MessageHeader.cpp
  src/MessageMember.cpp
  src/MessageSerializer.cpp
  src/MessageTraits.cpp
  src/MessageType.cpp
  src/MessageTypeParser.cpp
  src/MessageVariable.cpp
  src/MessageVariant.cpp
  src/Publisher.cpp
  src/Serialization.cpp
  src/Serializer.cpp
  src/Subscriber.cpp
  src/Variant.cpp
)

add_executable(
  echo
  src/echo.cpp
)
add_executable(
  info
  src/info.cpp
)
add_executable(
  publish
  src/publish.cpp
)
add_executable(
  relay
  src/relay.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(
  variant_topic_tools
  ${catkin_LIBRARIES}
)

target_link_libraries(
  echo
  variant_topic_tools
  ${catkin_LIBRARIES}
)
target_link_libraries(
  info
  variant_topic_tools
  ${catkin_LIBRARIES}
)
target_link_libraries(
  publish
  variant_topic_tools
  ${catkin_LIBRARIES}
)
target_link_libraries(
  relay
  variant_topic_tools
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS ${PROJECT_NAME} echo info publish relay
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.hpp"
  PATTERN "*.h"
  PATTERN "*.tpp"
)

##########
## Test ##
##########
if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    std_msgs
  )

  catkin_add_gtest(test_${PROJECT_NAME}
    test/variant_topic_test.cpp
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
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test
  )

  if(TARGET test_${PROJECT_NAME})
    target_link_libraries( test_${PROJECT_NAME}
      ${PROJECT_NAME}
      ${catkin_LIBRARIES}
    )
    find_package(cmake_code_coverage QUIET)
    if(cmake_code_coverage_FOUND)
      add_gtest_coverage()
    endif(cmake_code_coverage_FOUND)
  endif()
endif()
