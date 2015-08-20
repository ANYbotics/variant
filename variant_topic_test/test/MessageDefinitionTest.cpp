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
 * along with this program. If not, see <http://www.gnu.org/licenses/>.       *.
 ******************************************************************************/

#include <gtest/gtest.h>

#include <variant_msgs/Test.h>

#include <variant_topic_tools/DataTypeRegistry.h>
#include <variant_topic_tools/MessageDefinition.h>

using namespace variant_topic_tools;

TEST(MessageDefinition, Parse) {
  DataTypeRegistry registry;
  
  MessageDefinition d1 = MessageDefinition::create<variant_msgs::Test>();
  MessageDataType t1 = d1.getMessageDataType();
  MessageType m2("my_msgs/Test", "*",
    ros::message_traits::definition<variant_msgs::Test>());
  MessageDefinition d2(m2);
  MessageDataType t2 = d2.getMessageDataType();
  
  EXPECT_TRUE(t1.isValid());
  EXPECT_EQ("header", t1[0].getName());
  EXPECT_TRUE(t1[1].isConstant());
  EXPECT_TRUE(t1[2].isVariable());
  EXPECT_TRUE(t1[2].getType().isBuiltin());
  EXPECT_TRUE(t1[4].getType().isMessage());

  EXPECT_NO_THROW(d1.getField("header"));
  EXPECT_NO_THROW(d1.getField("string/data"));
  EXPECT_ANY_THROW(d1.getField("string/length"));
  EXPECT_TRUE(d1.hasField("byte_constant"));
  EXPECT_EQ("std_msgs/Header", d1["header"].getValue().getIdentifier());
  
  registry.clear();
}

TEST(MessageDefinition, Load) {
  DataTypeRegistry registry;
  
  MessageDefinition d1;
  EXPECT_NO_THROW(d1.load("variant_msgs/Test"));
  EXPECT_NO_THROW(d1.getField("header"));
  EXPECT_NO_THROW(d1.getField("string/data"));
  EXPECT_ANY_THROW(d1.getField("string/length"));
  EXPECT_TRUE(d1.hasField("byte_constant"));
  EXPECT_EQ("std_msgs/Header", d1["header"].getValue().getIdentifier());
  
  registry.clear();
}
