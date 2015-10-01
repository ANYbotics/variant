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

#include <variant_msgs/Test.h>

#include <variant_topic_tools/DataTypeRegistry.h>
#include <variant_topic_tools/MessageDefinition.h>
#include <variant_topic_tools/Variant.h>
#include <variant_topic_tools/VariantMessage.h>

using namespace variant_topic_tools;

TEST(Variant, Builtin) {
  Variant v1 = DataType("float64").createVariant();
  
  EXPECT_TRUE(v1.hasType());
  EXPECT_FALSE(v1.isEmpty());
  EXPECT_EQ(0.0, v1.getValue<double>());
  EXPECT_FALSE(v1.isEmpty());
  EXPECT_ANY_THROW(v1.getValue<int>());
  EXPECT_ANY_THROW(v1.setValue(0));
  
  v1 = M_PI;

  EXPECT_EQ(M_PI, v1.getValue<double>());
  EXPECT_TRUE(v1 == M_PI);
  EXPECT_FALSE(v1 != M_PI);
  EXPECT_TRUE(v1 != 1.0);
  EXPECT_FALSE(v1 == 1.0);
  
  v1 = 1.0;
  
  EXPECT_TRUE(v1 != 1);
  EXPECT_FALSE(v1 == 1);
  
  v1.clear();
  
  EXPECT_FALSE(v1.hasType());
  EXPECT_TRUE(v1.isEmpty());
}

TEST(Variant, Message) {
  DataTypeRegistry registry;
  
//   VariantMessage v1(MessageDefinition::create<std_msgs::Bool>().
//     getMessageDataType());
//   VariantMessage v2(MessageDefinition::create<std_msgs::String>().
//     getMessageDataType());
//   VariantMessage v3(MessageDefinition::create<variant_msgs::Test>().
//     getMessageDataType());
  
//   EXPECT_TRUE(v1.hasType());
//   EXPECT_FALSE(v1.isEmpty());
//   EXPECT_EQ(1, v1.getNumMembers());
//   EXPECT_TRUE(v1["data"].hasType());
//   EXPECT_FALSE(v1["data"].isEmpty());
//   EXPECT_NO_THROW(v1["data"] = true);
//   EXPECT_EQ(true, v1["data"].getValue<bool>());
  
//   EXPECT_TRUE(v2.hasType());
//   EXPECT_FALSE(v2.isEmpty());
//   EXPECT_EQ(1, v2.getNumMembers());
//   EXPECT_TRUE(v2["data"].hasType());
//   EXPECT_FALSE(v2["data"].isEmpty());
//   EXPECT_NO_THROW(v2["data"] = std::string("Test"));
//   EXPECT_EQ("Test", v2["data"].getValue<std::string>());
  
//   EXPECT_TRUE(v3.hasType());
//   EXPECT_FALSE(v3.isEmpty());
//   EXPECT_TRUE(v3["header"].hasType());
//   EXPECT_FALSE(v3["header"].isEmpty());
//   EXPECT_NO_THROW(v3["builtin_string"] = std::string("Test"));
//   EXPECT_EQ("Test", v3["builtin_string"].getValue<std::string>());
  
  registry.clear();
}
