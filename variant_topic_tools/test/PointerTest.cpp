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

#include <variant_topic_tools/ArrayMemberPointer.h>
#include <variant_topic_tools/BuiltinPointer.h>
#include <variant_topic_tools/MessageMemberPointer.h>

using namespace variant_topic_tools;

TEST(Pointer, Builtin) {
  BuiltinPointer<int> p1(new int());
  BuiltinPointer<int> p2;

  EXPECT_TRUE(p1);
  EXPECT_NO_THROW(*p1);

  EXPECT_FALSE(p2);
  EXPECT_ANY_THROW(*p2);
}

TEST(Pointer, Array) {
  auto* a1 = new boost::array<int, 3>();
  (*a1)[0] = 0;
  (*a1)[1] = 1;
  (*a1)[2] = 2;
  ArrayMemberPointer<int[3]> p1(a1, 1);

  auto* a2 = new std::vector<int>(3);
  (*a2)[0] = 0;
  (*a2)[1] = 1;
  (*a2)[2] = 2;
  ArrayMemberPointer<int[]> p2(a2, 1);
  ArrayMemberPointer<int[]> p3(p2.getArray(), 2);

  EXPECT_EQ(a1, p1.getArray().get());
  EXPECT_EQ(1, p1.getIndex());
  EXPECT_EQ(1, *p1);

  EXPECT_EQ(a2, p2.getArray().get());
  EXPECT_EQ(1, p2.getIndex());
  EXPECT_EQ(1, *p2);

  EXPECT_EQ(a2, p3.getArray().get());
  EXPECT_EQ(2, p3.getIndex());
  EXPECT_EQ(2, *p3);
}

TEST(Pointer, Message) {
  auto* m1 = new variant_msgs::Test();
  m1->builtin_int = 42;
  m1->builtin_string = "Test";

  MessageMemberPointer<variant_msgs::Test, int32_t> p1(m1, offsetof(variant_msgs::Test, builtin_int));
  MessageMemberPointer<variant_msgs::Test, std::string> p2(p1.getMessage(), offsetof(variant_msgs::Test, builtin_string));

  EXPECT_EQ(m1, p1.getMessage().get());
  EXPECT_EQ(offsetof(variant_msgs::Test, builtin_int), p1.getOffset());
  EXPECT_EQ(m1->builtin_int, *p1);

  EXPECT_EQ(m1, p2.getMessage().get());
  EXPECT_EQ(offsetof(variant_msgs::Test, builtin_string), p2.getOffset());
  EXPECT_EQ(m1->builtin_string, *p2);
}

TEST(Pointer, ArrayOfMessages) {
  auto* m1 = new variant_msgs::Test();
  m1->string_array[1].data = "Test";

  MessageMemberPointer<variant_msgs::Test, std_msgs::String[3]> p1(m1, offsetof(variant_msgs::Test, string_array));
  ArrayMemberPointer<std_msgs::String[3]> p2(p1, 1);
  MessageMemberPointer<std_msgs::String, std::string> p3(p2, offsetof(std_msgs::String, data));

  EXPECT_EQ(m1, p1.getMessage().get());
  EXPECT_EQ(offsetof(variant_msgs::Test, string_array), p1.getOffset());

  EXPECT_EQ(&m1->string_array, p2.getArray().get());
  EXPECT_EQ(1, p2.getIndex());

  EXPECT_EQ(&m1->string_array[1], p3.getMessage().get());
  EXPECT_EQ(offsetof(std_msgs::String, data), p3.getOffset());

  EXPECT_NO_THROW(p1.reset());
  EXPECT_ANY_THROW(*p1);
  EXPECT_NO_THROW(*p2);
  EXPECT_NO_THROW(*p3);
  EXPECT_EQ("Test", *p3);
  EXPECT_NO_THROW(p2.reset());
  EXPECT_ANY_THROW(*p2);
  EXPECT_NO_THROW(*p3);
  EXPECT_EQ("Test", *p3);
  EXPECT_NO_THROW(p3.reset());
}
