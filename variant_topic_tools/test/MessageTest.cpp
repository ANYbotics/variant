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

#include <geometry_msgs/PoseStamped.h>

#include <variant_msgs/Test.h>

#include <variant_topic_tools/DataTypeRegistry.h>
#include <variant_topic_tools/Message.h>
#include <variant_topic_tools/MessageVariant.h>

using namespace variant_topic_tools;

TEST(Message, Conversion) {
  DataTypeRegistry registry;
  
  variant_msgs::Test m1;
  m1.builtin_int = 42;
  m1.builtin_string = "Test";
  Message m2 = m1;
  variant_msgs::Test::Ptr m3 = m2.toMessage<variant_msgs::Test>();
    
  EXPECT_EQ(ros::message_traits::datatype<variant_msgs::Test>(),
    m2.getType().getDataType());
  EXPECT_EQ(ros::message_traits::md5sum<variant_msgs::Test>(),
    m2.getType().getMD5Sum());
  EXPECT_EQ(ros::message_traits::definition<variant_msgs::Test>(),
    m2.getType().getDefinition());
  EXPECT_EQ(m1.builtin_int, m3->builtin_int);
  EXPECT_EQ(m1.builtin_string, m3->builtin_string);
  EXPECT_ANY_THROW(m2.toMessage<std_msgs::Bool>());
  
  geometry_msgs::PoseStamped m4;
  Message m5 = m4;
  EXPECT_EQ(ros::message_traits::datatype<geometry_msgs::PoseStamped>(),
    m5.getType().getDataType());
  EXPECT_EQ(ros::message_traits::md5sum<geometry_msgs::PoseStamped>(),
    m5.getType().getMD5Sum());
  EXPECT_EQ(ros::message_traits::definition<geometry_msgs::PoseStamped>(),
    m5.getType().getDefinition());
  
  registry.clear();
}

TEST(Message, Serialization) {
  DataTypeRegistry registry;
  
  variant_msgs::Test m1, m2;
  m1.builtin_int = 42;
  m1.builtin_string = "Test";
  Message m3 = m1;
  MessageVariant v1;
  
  EXPECT_NO_THROW(m3.deserialize(v1));
  EXPECT_EQ(m1.builtin_int, v1["builtin_int"].getValue<int>());
  EXPECT_EQ(m1.builtin_string, v1["builtin_string"].getValue<std::string>());
  
  EXPECT_NO_THROW(m3.serialize(v1));
  EXPECT_EQ(ros::serialization::serializationLength<variant_msgs::Test>(m1),
    m3.getData().size());
  EXPECT_NO_THROW(m2 = *m3.toMessage<variant_msgs::Test>());
  EXPECT_EQ(m1.builtin_int, m2.builtin_int);
  EXPECT_EQ(m1.builtin_string, m2.builtin_string);
  
  registry.clear();
}
