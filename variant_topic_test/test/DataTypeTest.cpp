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

#include <variant_topic_tools/ArrayDataType.h>
#include <variant_topic_tools/BuiltinDataType.h>
#include <variant_topic_tools/DataTypeRegistry.h>
#include <variant_topic_tools/MessageDataType.h>

using namespace variant_topic_tools;

TEST(DataType, Array) {
  DataTypeRegistry registry;
  registry.addArrayDataType<int32_t, 3>();
  registry.addArrayDataType<int32_t, 0>();
  
  ArrayDataType a1("int32[3]");
  ArrayDataType a2("int32[]");

  EXPECT_TRUE(a1.isValid());
  EXPECT_TRUE(a1.isArray());
  EXPECT_TRUE(a1.isFixedSize());
  
  EXPECT_TRUE(a2.isValid());
  EXPECT_TRUE(a2.isArray());
  EXPECT_FALSE(a2.isFixedSize());
  
  EXPECT_TRUE(registry.getDataType<int32_t[3]>().isArray());
  EXPECT_TRUE(registry.getDataType<int32_t[]>().isArray());
  
  registry.clear();
}

TEST(DataType, Builtin) {
  BuiltinDataType b1("int32");

  EXPECT_TRUE(b1.isValid());
  EXPECT_TRUE(b1.isBuiltin());
  EXPECT_TRUE(b1.hasTypeInfo());
  EXPECT_EQ(typeid(int32_t), b1.getTypeInfo());
  EXPECT_TRUE(b1.isFixedSize());
  EXPECT_EQ(sizeof(int32_t), b1.getSize());
  EXPECT_FALSE(b1.createVariant().isEmpty());
  EXPECT_TRUE(b1.isNumeric());
}

TEST(DataType, Message) {
  DataTypeRegistry registry;
  
  registry.addArrayDataType<double, 0>();
  registry.addArrayDataType<double, 3>();
  
  MessageDataType m1 = registry.addMessageDataType<std_msgs::Bool>();
  MessageDataType m2 = registry.addMessageDataType("my_msgs/Double");
  m2.addVariableMember<double>("data");
  MessageDataType m3 = registry.addMessageDataType("my_msgs/Complex",
    "float64 real\n"
    "float64 imaginary\n"
  );
  MessageDataType m4 = registry.addMessageDataType("my_msgs/Vector",
    "float64[] data\n");
  MessageDataType m5 = registry.addMessageDataType("my_msgs/Array",
    "float64[3] data\n");
  
  EXPECT_TRUE(m1.isValid());
  EXPECT_TRUE(m1.isMessage());
  EXPECT_TRUE(m1.hasTypeInfo());
  EXPECT_EQ(typeid(std_msgs::Bool), m1.getTypeInfo());
  EXPECT_TRUE(m2.isValid());
  EXPECT_TRUE(m2.isMessage());
  EXPECT_FALSE(m2.hasTypeInfo());
  EXPECT_TRUE(m3.isValid());
  EXPECT_TRUE(m3.isMessage());
  EXPECT_FALSE(m3.hasTypeInfo());
  EXPECT_TRUE(m4.isValid());
  EXPECT_TRUE(m4.isMessage());
  EXPECT_FALSE(m4.hasTypeInfo());
  EXPECT_TRUE(m5.isValid());
  EXPECT_TRUE(m5.isMessage());
  EXPECT_FALSE(m5.hasTypeInfo());
  
  registry.clear();
}
