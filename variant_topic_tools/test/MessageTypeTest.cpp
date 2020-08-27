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

#include <geometry_msgs/PoseStamped.h>

#include <variant_msgs/Test.h>

#include <variant_topic_tools/DataTypeRegistry.h>
#include <variant_topic_tools/MessageType.h>

using namespace variant_topic_tools;

TEST(MessageType, Load) {
  DataTypeRegistry registry;
  
  MessageType t1, t2, t3;
  
  EXPECT_NO_THROW(t1.load("variant_msgs/Test"));
  EXPECT_EQ(ros::message_traits::definition<variant_msgs::Test>(),
    t1.getDefinition());
  EXPECT_ANY_THROW(t2.load("variant_msgs/Undefined"));  
  EXPECT_NO_THROW(t3.load("geometry_msgs/PoseStamped"));
  EXPECT_EQ(ros::message_traits::definition<geometry_msgs::PoseStamped>(),
    t3.getDefinition());
  
  registry.clear();
}
