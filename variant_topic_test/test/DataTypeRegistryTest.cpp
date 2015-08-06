/******************************************************************************
 * Copyright (C) 2014 by Ralf Kaestner                                        *
 * ralf.kaestner@gmail.com                                                    *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify       *
 * it under the terms of the Lesser GNU General Public License as published by*
 * the Free Software Foundation; either version 3 of the License, or          *
 * (at your option) any later version.                                        *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the               *
 * Lesser GNU General Public License for more details.                        *
 *                                                                            *
 * You should have received a copy of the Lesser GNU General Public License   *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.       *
 ******************************************************************************/

#include <gtest/gtest.h>

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <variant_topic_tools/DataTypeRegistry.h>

using namespace variant_topic_tools;

TEST (DataTypeRegistry, Builtin) {
  DataTypeRegistry registry;

  EXPECT_TRUE(registry.getDataType<int8_t>().isBuiltin());
  EXPECT_TRUE(registry.getDataType("int8").isBuiltin());
  EXPECT_TRUE(registry["int8"].isBuiltin());
  EXPECT_TRUE(registry[typeid(int8_t)].isBuiltin());
}

TEST (DataTypeRegistry, Array) {
  DataTypeRegistry registry;

  registry.addArrayDataType<int, 3>();
  registry.addArrayDataType<int, 0>();
  registry.addArrayDataType<boost::array<double, 3> >();
  registry.addArrayDataType<std::vector<double> >();
  registry.addArrayDataType("int8", 3);
  registry.addArrayDataType(typeid(int8_t));
  
  typedef boost::array<int, 3> boost_array_int_3;
  typedef boost::array<double, 3> boost_array_double_3;
  typedef boost::array<int8_t, 3> boost_array_int8_t_3;
  
  EXPECT_TRUE(registry.getDataType<boost_array_int_3>().isArray());
  EXPECT_TRUE(registry.getDataType("int32[3]").isArray());
  EXPECT_TRUE(registry.getDataType<std::vector<int> >().isArray());
  EXPECT_TRUE(registry.getDataType("int32[]").isArray());
  EXPECT_TRUE(registry.getDataType<boost_array_double_3>().isArray());
  EXPECT_TRUE(registry.getDataType("float64[3]").isArray());
  EXPECT_TRUE(registry.getDataType<std::vector<double> >().isArray());
  EXPECT_TRUE(registry.getDataType("float64[]").isArray());
  EXPECT_TRUE(registry.getDataType<boost_array_int8_t_3>().isArray());
  EXPECT_TRUE(registry.getDataType("int8[3]").isArray());
  EXPECT_TRUE(registry.getDataType<std::vector<int8_t> >().isArray());
  EXPECT_TRUE(registry.getDataType("int8[]").isArray());
}

TEST (DataTypeRegistry, Message) {
  DataTypeRegistry registry;

  registry.addMessageDataType<std_msgs::String>();
  registry.addMessageDataType(ros::message_traits::datatype<std_msgs::Bool>(),
    ros::message_traits::definition<std_msgs::Bool>());
  registry.addMessageDataType("my_msgs/Double").addVariable<double>("data");
  registry.addMessageDataType("my_msgs/Complex",
    "float64 real\n"
    "float64 imaginary\n"
  );
  
  EXPECT_TRUE(registry.getDataType<std_msgs::String>().isMessage());
  EXPECT_TRUE(registry.getDataType(
    ros::message_traits::datatype<std_msgs::String>()).isMessage());
  EXPECT_TRUE(registry.getDataType<std_msgs::Bool>().isMessage());
  EXPECT_TRUE(registry.getDataType("my_msgs/Double").isMessage());
  EXPECT_TRUE(registry.getDataType("my_msgs/Complex").isMessage());
}
