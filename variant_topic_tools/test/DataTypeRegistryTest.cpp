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

TEST(DataTypeRegistry, Array) {
  DataTypeRegistry registry;

  ArrayDataType a1 = registry.addArrayDataType<int, 3>();
  ArrayDataType a2 = registry.addArrayDataType<int, 0>();
  ArrayDataType a3 = registry.addArrayDataType<double[3]>();
  ArrayDataType a4 = registry.addArrayDataType<double[]>();
  ArrayDataType a5 = registry.addArrayDataType<bool[3]>();
  ArrayDataType a6 = registry.addArrayDataType<bool[]>();
  ArrayDataType a7 = registry.addArrayDataType("int8", 3);
  ArrayDataType a8 = registry.addArrayDataType(typeid(int8_t));

  EXPECT_TRUE(registry.getDataType<int[3]>().isArray());
  EXPECT_TRUE(registry.getDataType("int32[3]").isArray());
  EXPECT_TRUE(registry.getDataType<int[]>().isArray());
  EXPECT_TRUE(registry.getDataType("int32[]").isArray());
  EXPECT_TRUE(registry.getDataType<double[3]>().isArray());
  EXPECT_TRUE(registry.getDataType("float64[3]").isArray());
  EXPECT_TRUE(registry.getDataType<double[]>().isArray());
  EXPECT_TRUE(registry.getDataType("float64[]").isArray());
  EXPECT_TRUE(registry.getDataType<bool[3]>().isArray());
  EXPECT_TRUE(registry.getDataType("bool[3]").isArray());
  EXPECT_TRUE(registry.getDataType<bool[]>().isArray());
  EXPECT_TRUE(registry.getDataType("bool[]").isArray());
  EXPECT_TRUE(registry.getDataType<int8_t[3]>().isArray());
  EXPECT_TRUE(registry.getDataType("int8[3]").isArray());
  EXPECT_TRUE(registry.getDataType<int8_t[]>().isArray());
  EXPECT_TRUE(registry.getDataType("int8[]").isArray());
  EXPECT_FALSE(const_cast<const DataTypeRegistry&>(registry).getDataType("uint8[3]").isArray());
  EXPECT_TRUE(registry.getDataType("uint8[3]").isArray());
  EXPECT_FALSE(const_cast<const DataTypeRegistry&>(registry).getDataType("uint8[]").isArray());
  EXPECT_TRUE(registry.getDataType("uint8[]").isArray());

  variant_topic_tools::DataTypeRegistry::clear();
}

TEST(DataTypeRegistry, Builtin) {
  DataTypeRegistry registry;

  EXPECT_TRUE(registry.getDataType<int8_t>().isBuiltin());
  EXPECT_TRUE(registry.getDataType("int8").isBuiltin());
  EXPECT_TRUE(registry["int8"].isBuiltin());
  EXPECT_TRUE(registry[typeid(int8_t)].isBuiltin());
}

TEST(DataTypeRegistry, Message) {
  DataTypeRegistry registry;

  MessageDataType m1 = registry.addMessageDataType<std_msgs::String>();
  MessageDataType m2 =
      registry.addMessageDataType(ros::message_traits::datatype<std_msgs::Bool>(), ros::message_traits::definition<std_msgs::Bool>());
  MessageDataType m3 = registry.addMessageDataType("my_msgs/Double");
  m3.addVariableMember<double>("data");

  EXPECT_TRUE(registry.getDataType<std_msgs::String>().isMessage());
  EXPECT_TRUE(registry.getDataType(ros::message_traits::datatype<std_msgs::String>()).isMessage());
  EXPECT_TRUE(registry.getDataType<std_msgs::Bool>().isMessage());
  EXPECT_TRUE(registry.getDataType("my_msgs/Double").isMessage());

  variant_topic_tools::DataTypeRegistry::clear();
}
