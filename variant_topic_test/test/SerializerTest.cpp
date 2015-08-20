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

#include <variant_topic_tools/ArraySerializer.h>
#include <variant_topic_tools/BuiltinSerializer.h>
#include <variant_topic_tools/DataTypeRegistry.h>
#include <variant_topic_tools/MessageDefinition.h>
#include <variant_topic_tools/MessageSerializer.h>

using namespace variant_topic_tools;

TEST(Serializer, Array) {
  DataTypeRegistry registry;

  std::vector<uint8_t> d1(1024);
  ros::serialization::OStream o1(d1.data(), d1.size());
  ros::serialization::IStream i1(d1.data(), d1.size());

  typedef boost::array<int, 3> boost_array_int_3;
  typedef std::vector<int32_t> std_vector_int;
  
  boost_array_int_3 a1;
  a1[0] = 0; a1[1] = 1; a1[2] = 2;
  std_vector_int a2(3);
  a2[0] = 0; a2[1] = 1; a2[2] = 2;
  Variant v1, v2;
  
  Serializer s1 = registry.getDataType<boost_array_int_3>().createSerializer();
  Serializer s2 = registry.getDataType<std_vector_int>().createSerializer();
  
  s1.serialize(o1, a1);
  EXPECT_EQ(d1.size()-ros::serialization::serializationLength(a1),
    o1.getLength());
  s1.deserialize(i1, v1);
  EXPECT_EQ(a1[0], v1.getValue<boost_array_int_3>()[0]);
  EXPECT_EQ(a1[1], v1.getValue<boost_array_int_3>()[1]);
  EXPECT_EQ(a1[2], v1.getValue<boost_array_int_3>()[2]);
  s2.serialize(o1, a2);
  s2.deserialize(i1, v2);
  EXPECT_EQ(a2[0], v2.getValue<std_vector_int>()[0]);
  EXPECT_EQ(a2[1], v2.getValue<std_vector_int>()[1]);
  EXPECT_EQ(a2[2], v2.getValue<std_vector_int>()[2]);
  
  registry.clear();
}

TEST(Serializer, Builtin) {
  DataTypeRegistry registry;

  std::vector<uint8_t> d1(1024);
  ros::serialization::OStream o1(d1.data(), d1.size());
  ros::serialization::IStream i1(d1.data(), d1.size());
  
  int32_t b1 = 42;
  double b2 = 42.0;
  Variant v1, v2;
  
  Serializer s1 = registry.getDataType<int32_t>().createSerializer();
  Serializer s2 = registry.getDataType<double>().createSerializer();
  Serializer s3 = registry.getDataType<bool>().createSerializer();
  
  s1.serialize(o1, b1);
  EXPECT_EQ(d1.size()-sizeof(b1), o1.getLength());
  s1.deserialize(i1, v1);
  EXPECT_EQ(b1, v1.getValue<int32_t>());
  s2.serialize(o1, b2);
  s2.deserialize(i1, v2);
  EXPECT_EQ(b2, v2.getValue<double>());
  EXPECT_ANY_THROW(s3.serialize(o1, b1));
}

TEST(Serializer, Message) {
  DataTypeRegistry registry;

  std::vector<uint8_t> d1(1024);
  ros::serialization::OStream o1(d1.data(), d1.size());
  ros::serialization::IStream i1(d1.data(), d1.size());
  ros::Time::init();

  std_msgs::Bool m1;
  m1.data = true;
  std_msgs::String m2;
  m2.data = "Test";
  variant_msgs::Test m3;
  m3.header.stamp = ros::Time::now();
  m3.builtin_int = 42;
  m3.builtin_string = "Test";
  m3.string.data = "Test";
  m3.builtin_int_array[0] = 0;
  m3.builtin_int_array[1] = 1;
  m3.builtin_int_array[2] = 2;
  m3.builtin_int_vector.resize(3);
  m3.builtin_int_vector[0] = 0;
  m3.builtin_int_vector[1] = 1;
  m3.builtin_int_vector[2] = 2;
  Variant v1, v2, v3;
  
  Serializer s1 = registry.getDataType<std_msgs::Bool>().createSerializer();
  Serializer s2 = registry.getDataType<std_msgs::String>().createSerializer();
  Serializer s3 = registry.getDataType<variant_msgs::Test>().
    createSerializer();
  
  s1.serialize(o1, m1);
  EXPECT_EQ(d1.size()-ros::serialization::serializationLength(m1),
    o1.getLength());
  s1.deserialize(i1, v1);
  EXPECT_EQ(m1.data, v1.getValue<std_msgs::Bool>().data);
  s2.serialize(o1, m2);
  s2.deserialize(i1, v2);
  EXPECT_EQ(m2.data, v2.getValue<std_msgs::String>().data);
  s3.serialize(o1, m3);
  s3.deserialize(i1, v3);
  EXPECT_EQ(m3.header.stamp, v3.getValue<variant_msgs::Test>().header.stamp);
  EXPECT_EQ(m3.builtin_int, v3.getValue<variant_msgs::Test>().builtin_int);
  EXPECT_EQ(m3.builtin_string,
    v3.getValue<variant_msgs::Test>().builtin_string);
  EXPECT_EQ(m3.string.data, v3.getValue<variant_msgs::Test>().string.data);
  EXPECT_EQ(m3.builtin_int_array[0],
    v3.getValue<variant_msgs::Test>().builtin_int_array[0]);
  EXPECT_EQ(m3.builtin_int_array[1],
    v3.getValue<variant_msgs::Test>().builtin_int_array[1]);
  EXPECT_EQ(m3.builtin_int_array[2],
    v3.getValue<variant_msgs::Test>().builtin_int_array[2]);
  EXPECT_EQ(m3.builtin_int_vector[0],
    v3.getValue<variant_msgs::Test>().builtin_int_vector[0]);
  EXPECT_EQ(m3.builtin_int_vector[1],
    v3.getValue<variant_msgs::Test>().builtin_int_vector[1]);
  EXPECT_EQ(m3.builtin_int_vector[2],
    v3.getValue<variant_msgs::Test>().builtin_int_vector[2]);
  
  registry.clear();
}

TEST(Serializer, VariantMessage) {
  DataTypeRegistry registry;

  std::vector<uint8_t> d1(1024);
  ros::serialization::OStream o1(d1.data(), d1.size());
  ros::serialization::IStream i1(d1.data(), d1.size());
  ros::Time::init();

  variant_msgs::Test m1;
  m1.header.stamp = ros::Time::now();
  m1.builtin_int = 42;
  m1.builtin_string = "Test";
  m1.string.data = "Test";
  m1.builtin_int_array[0] = 0;
  m1.builtin_int_array[1] = 1;
  m1.builtin_int_array[2] = 2;
  m1.builtin_int_vector.resize(3);
  m1.builtin_int_vector[0] = 0;
  m1.builtin_int_vector[1] = 1;
  m1.builtin_int_vector[2] = 2;
  Variant v1;
  
  MessageType t1("my_msgs/Test", "*",
    ros::message_traits::definition<variant_msgs::Test>());
  Serializer s1 = MessageDefinition(t1).getMessageDataType().
    createSerializer();
    
  ros::serialization::serialize(o1, m1);
  
  registry.clear();
}
