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

  boost::array<int, 3> a1;
  a1[0] = 0; a1[1] = 1; a1[2] = 2;
  std::vector<int> a2(3);
  a2[0] = 0; a2[1] = 1; a2[2] = 2;
  Variant v1, v2;

  Serializer s1 = registry.getDataType<int[3]>().createSerializer();
  Serializer s2 = registry.getDataType<int[]>().createSerializer();

  s1.serialize(o1, a1);
  EXPECT_EQ(d1.size()-ros::serialization::serializationLength(a1),
    o1.getLength());
  s1.deserialize(i1, v1);
  EXPECT_EQ(a1[0], v1.getValue<int[3]>()[0]);
  EXPECT_EQ(a1[1], v1.getValue<int[3]>()[1]);
  EXPECT_EQ(a1[2], v1.getValue<int[3]>()[2]);
  s2.serialize(o1, a2);
  s2.deserialize(i1, v2);
  EXPECT_EQ(a2[0], v2.getValue<int[]>()[0]);
  EXPECT_EQ(a2[1], v2.getValue<int[]>()[1]);
  EXPECT_EQ(a2[2], v2.getValue<int[]>()[2]);

  registry.clear();
}

TEST(Serializer, Builtin) {
  DataTypeRegistry registry;

  std::vector<uint8_t> d1(1024);
  ros::serialization::OStream o1(d1.data(), d1.size());
  ros::serialization::IStream i1(d1.data(), d1.size());

  int b1 = 42;
  double b2 = 42.0;
  Variant v1, v2;

  Serializer s1 = registry.getDataType<int>().createSerializer();
  Serializer s2 = registry.getDataType<double>().createSerializer();
  Serializer s3 = registry.getDataType<bool>().createSerializer();

  s1.serialize(o1, b1);
  EXPECT_EQ(d1.size()-sizeof(b1), o1.getLength());
  s1.deserialize(i1, v1);
  EXPECT_EQ(b1, v1.getValue<int>());
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

TEST(Serializer, VariantArray) {
  DataTypeRegistry registry;

  std::vector<uint8_t> d1(1024);
  ros::serialization::OStream o1(d1.data(), d1.size());
  ros::serialization::IStream i1(d1.data(), d1.size());
  std::vector<uint8_t> d2(1024);
  ros::serialization::OStream o2(d2.data(), d2.size());
  ros::serialization::IStream i2(d2.data(), d2.size());

  boost::array<int, 3> a1;
  a1[0] = 0; a1[1] = 1; a1[2] = 2;
  std::vector<int> a2(3);
  a2[0] = 0; a2[1] = 1; a2[2] = 2;

  ArrayVariant v1 = registry.getDataType("int32[3]").createVariant();
  v1[0] = a1[0]; v1[1] = a1[1]; v1[2] = a1[2];
  ArrayVariant v2 = registry.getDataType("int32[3]").createVariant();
  ArrayVariant v3 = registry.getDataType("int32[]").createVariant();
  v3 += a2[0]; v3 += a2[1]; v3 += a2[2];
  ArrayVariant v4 = registry.getDataType("int32[]").createVariant();

  Serializer s1 = v1.createSerializer();
  Serializer s2 = v3.createSerializer();

  s1.serialize(o1, v1);
  EXPECT_EQ(d1.size()-v1.getType().getSize(), o1.getLength());
  s1.deserialize(i1, v2);
  EXPECT_EQ(v1, v2);
  ros::serialization::serialize(o2, a1);
  s1.deserialize(i2, v2);
  EXPECT_EQ(a1[0], v2[0].getValue<int>());
  EXPECT_EQ(a1[1], v2[1].getValue<int>());
  EXPECT_EQ(a1[2], v2[2].getValue<int>());
  s1.serialize(o2, v1);
  ros::serialization::deserialize(i2, a1);
  EXPECT_EQ(v1[0].getValue<int>(), a1[0]);
  EXPECT_EQ(v1[1].getValue<int>(), a1[1]);
  EXPECT_EQ(v1[2].getValue<int>(), a1[2]);

  s2.serialize(o1, v3);
  s2.deserialize(i1, v4);
  EXPECT_EQ(v3, v4);
  ros::serialization::serialize(o2, a2);
  s2.deserialize(i2, v4);
  EXPECT_EQ(a2.size(), v4.getNumMembers());
  EXPECT_EQ(a2[0], v4[0].getValue<int>());
  EXPECT_EQ(a2[1], v4[1].getValue<int>());
  EXPECT_EQ(a2[2], v4[2].getValue<int>());
  s2.serialize(o2, v3);
  ros::serialization::deserialize(i2, a2);
  EXPECT_EQ(v3.getNumMembers(), a2.size());
  EXPECT_EQ(v3[0].getValue<int>(), a2[0]);
  EXPECT_EQ(v3[1].getValue<int>(), a2[1]);
  EXPECT_EQ(v3[2].getValue<int>(), a2[2]);

  registry.clear();
}

TEST(Serializer, VariantMessage) {
  DataTypeRegistry registry;

  std::vector<uint8_t> d1(1024);
  ros::serialization::OStream o1(d1.data(), d1.size());
  ros::serialization::IStream i1(d1.data(), d1.size());
  std::vector<uint8_t> d2(1024);
  ros::serialization::OStream o2(d2.data(), d2.size());
  ros::serialization::IStream i2(d2.data(), d2.size());
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

  MessageDataType t1 = MessageDefinition::create<variant_msgs::Test>().
    getMessageDataType();
  MessageVariant v1 = t1.createVariant();
  v1["header/stamp"] = m1.header.stamp;
  v1["builtin_int"] = m1.builtin_int;
  v1["builtin_string"] = m1.builtin_string;
  v1["string/data"] = m1.string.data;
  v1["builtin_int_array/0"] = m1.builtin_int_array[0];
  v1["builtin_int_array/1"] = m1.builtin_int_array[1];
  v1["builtin_int_array/2"] = m1.builtin_int_array[2];
  v1["builtin_int_vector"].asArray().resize(3);
  v1["builtin_int_vector/0"] = m1.builtin_int_vector[0];
  v1["builtin_int_vector/1"] = m1.builtin_int_vector[1];
  v1["builtin_int_vector/2"] = m1.builtin_int_vector[2];
  MessageVariant v2 = t1.createVariant();

  Serializer s1 = v1.createSerializer();

  s1.serialize(o1, v1);
  s1.deserialize(i1, v2);
  EXPECT_EQ(v1, v2);

  ros::serialization::serialize(o2, m1);
  s1.deserialize(i2, v2);
  EXPECT_EQ(m1.header.stamp, v2["header/stamp"].getValue<ros::Time>());
  EXPECT_EQ(m1.builtin_int, v2["builtin_int"].getValue<int>());
  EXPECT_EQ(m1.builtin_string, v2["builtin_string"].getValue<std::string>());
  EXPECT_EQ(m1.string.data, v2["string/data"].getValue<std::string>());
  EXPECT_EQ(m1.builtin_int_array[0], v2["builtin_int_array/0"].
    getValue<int>());
  EXPECT_EQ(m1.builtin_int_array[1], v2["builtin_int_array/1"].
    getValue<int>());
  EXPECT_EQ(m1.builtin_int_array[2], v2["builtin_int_array/2"].
    getValue<int>());
  EXPECT_EQ(m1.builtin_int_vector.size(), v2["builtin_int_vector"].
    asArray().getNumMembers());
  EXPECT_EQ(m1.builtin_int_vector[0], v2["builtin_int_vector/0"].
    getValue<int>());
  EXPECT_EQ(m1.builtin_int_vector[1], v2["builtin_int_vector/1"].
    getValue<int>());
  EXPECT_EQ(m1.builtin_int_vector[2], v2["builtin_int_vector/2"].
    getValue<int>());
  s1.serialize(o2, v1);
  ros::serialization::deserialize(i2, m1);
  EXPECT_EQ(v1["header/stamp"].getValue<ros::Time>(), m1.header.stamp);
  EXPECT_EQ(v1["builtin_int"].getValue<int>(), m1.builtin_int);
  EXPECT_EQ(v1["builtin_string"].getValue<std::string>(), m1.builtin_string);
  EXPECT_EQ(v1["string/data"].getValue<std::string>(), m1.string.data);
  EXPECT_EQ(v1["builtin_int_array/0"].getValue<int>(),
    m1.builtin_int_array[0]);
  EXPECT_EQ(v1["builtin_int_array/1"].getValue<int>(),
    m1.builtin_int_array[1]);
  EXPECT_EQ(v1["builtin_int_array/2"].getValue<int>(),
    m1.builtin_int_array[2]);
  EXPECT_EQ(v1["builtin_int_vector"].asArray().getNumMembers(),
    m1.builtin_int_vector.size());
  EXPECT_EQ(v1["builtin_int_vector/0"].getValue<int>(),
    m1.builtin_int_vector[0]);
  EXPECT_EQ(v1["builtin_int_vector/1"].getValue<int>(),
    m1.builtin_int_vector[1]);
  EXPECT_EQ(v1["builtin_int_vector/2"].getValue<int>(),
    m1.builtin_int_vector[2]);

  registry.clear();
}
