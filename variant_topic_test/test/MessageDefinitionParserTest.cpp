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

#include <variant_topic_tools/MessageDefinitionParser.h>

using namespace variant_topic_tools;

TEST(MessageDefinitionParser, Parse) {
  std::vector<MessageType> messageTypes;
  
  MessageDefinitionParser::parse(
    ros::message_traits::datatype<variant_msgs::Test>(),
    ros::message_traits::definition<variant_msgs::Test>(),
    messageTypes);
  
  EXPECT_FALSE(messageTypes.empty());
  EXPECT_EQ(ros::message_traits::datatype<variant_msgs::Test>(),
    messageTypes[0].getDataType());
  EXPECT_EQ(ros::message_traits::datatype<std_msgs::Header>(),
    messageTypes[1].getDataType());
  EXPECT_EQ(ros::message_traits::definition<std_msgs::Header>(),
    messageTypes[1].getDefinition());
}

TEST(MessageDefinitionParser, Match) {
  std::string name, type, value;
  size_t size;
  
  EXPECT_TRUE(MessageDefinitionParser::matchType("int32"));
  EXPECT_TRUE(MessageDefinitionParser::matchArrayType("int32[3]",
    type, size));
  EXPECT_EQ("int32", type);
  EXPECT_EQ(3, size);
  EXPECT_TRUE(MessageDefinitionParser::matchArrayType("int32[]",
    type, size));
  EXPECT_EQ("int32", type);
  EXPECT_EQ(0, size);
  EXPECT_TRUE(MessageDefinitionParser::match("int32 x # Comment", name, type));
  EXPECT_EQ("x", name);
  EXPECT_EQ("int32", type);
  EXPECT_FALSE(MessageDefinitionParser::match("# Comment", name, type));
  EXPECT_FALSE(MessageDefinitionParser::match("int32 # Comment", name, type));
  EXPECT_TRUE(MessageDefinitionParser::match("int32 x=1 # Comment",
    name, type));
  EXPECT_EQ("x", name);
  EXPECT_EQ("int32", type);
  EXPECT_TRUE(MessageDefinitionParser::matchConstant("int32 x=1 # Comment",
    name, type, value));
  EXPECT_EQ("x", name);
  EXPECT_EQ("int32", type);
  EXPECT_EQ("1", value);
  EXPECT_TRUE(MessageDefinitionParser::matchConstant(
    "string x= value # Comment ", name, type, value));
  EXPECT_EQ("x", name);
  EXPECT_EQ("string", type);
  EXPECT_EQ("value # Comment", value);
  EXPECT_TRUE(MessageDefinitionParser::matchVariable("int32 x # Comment",
    name, type));
  EXPECT_EQ("x", name);
  EXPECT_EQ("int32", type);
  EXPECT_TRUE(MessageDefinitionParser::matchVariable("int32[3] x # Comment",
    name, type));
  EXPECT_EQ("x", name);
  EXPECT_EQ("int32[3]", type);
  EXPECT_TRUE(MessageDefinitionParser::matchArray("int32[3] x # Comment",
    name, type, size));
  EXPECT_EQ("x", name);
  EXPECT_EQ("int32", type);
  EXPECT_EQ(3, size);
  EXPECT_TRUE(MessageDefinitionParser::matchArray("int32[] x # Comment",
    name, type, size));
  EXPECT_EQ("x", name);
  EXPECT_EQ("int32", type);
  EXPECT_EQ(0, size);
}
