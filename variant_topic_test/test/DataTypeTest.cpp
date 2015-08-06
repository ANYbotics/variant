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
#include <variant_topic_tools/MessageDataType.h>

using namespace variant_topic_tools;

TEST (DataType, Array) {
  ArrayDataType a1("int32[3]");
  ArrayDataType a2("int32[]");

  EXPECT_TRUE(a1.isValid());
  EXPECT_TRUE(a1.isArray());
  EXPECT_TRUE(a1.isFixedSize());
  
  EXPECT_TRUE(a2.isValid());
  EXPECT_TRUE(a2.isArray());
  EXPECT_TRUE(a2.isFixedSize());
}

TEST (DataType, Builtin) {
  BuiltinDataType b1("int32");

  EXPECT_TRUE(b1.isValid());
  EXPECT_TRUE(b1.isBuiltin());
  EXPECT_TRUE(b1.hasTypeInfo());
  EXPECT_EQ(typeid(int32_t), b1.getTypeInfo());
  EXPECT_TRUE(b1.isFixedSize());
  EXPECT_EQ(sizeof(int32_t), b1.getSize());
  EXPECT_FALSE(b1.createVariant().isEmpty());
}

TEST (DataType, Message) {
  MessageDataType m1(ros::message_traits::definition<std_msgs::Bool>());
  
  EXPECT_TRUE(m1.isValid());
  EXPECT_TRUE(m1.isMessage());
}
