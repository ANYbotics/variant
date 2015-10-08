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

#include <variant_msgs/Test.h>

#include <variant_topic_tools/DataTypeRegistry.h>
#include <variant_topic_tools/MessageStream.h>

using namespace variant_topic_tools;

TEST(MessageStream, Output) {
  DataTypeRegistry registry;
  
  variant_msgs::Test m1;
  MessageStream s1(m1);
  
  ros::serialization::serialize(s1, m1);
  
  EXPECT_EQ(offsetof(variant_msgs::Test, header),
    s1.getMemberOffsets()[0]);
  EXPECT_EQ(registry.getDataType<std_msgs::Header>(),
    s1.getMemberTypes()[0]);
  EXPECT_EQ(offsetof(variant_msgs::Test, builtin_int),
    s1.getMemberOffsets()[1]);
  EXPECT_EQ(registry.getDataType<int32_t>(),
    s1.getMemberTypes()[1]);
  EXPECT_EQ(offsetof(variant_msgs::Test, builtin_string),
    s1.getMemberOffsets()[4]);
  EXPECT_EQ(registry.getDataType<std::string>(),
    s1.getMemberTypes()[4]);
  
  registry.clear();
}
