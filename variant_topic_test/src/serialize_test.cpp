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

#include <variant_topic_tools/ArraySerializer.h>
#include <variant_topic_tools/BuiltinSerializer.h>
#include <variant_topic_tools/DataTypeRegistry.h>
#include <variant_topic_tools/MessageSerializer.h>

using namespace variant_topic_tools;

int main(int argc, char **argv) {
  DataTypeRegistry registry;

  std::vector<uint8_t> d1(1024);
  ros::serialization::OStream o1(d1.data(), d1.size());
  ros::serialization::IStream i1(d1.data(), d1.size());
  
  int32_t b1 = 42;
  double b2 = 42.0;
  boost::array<int32_t, 3> a1;
  a1[0] = 0; a1[1] = 1; a1[2] = 2;
  std::vector<int32_t> a2(3);
  a2[0] = 0; a2[1] = 1; a2[2] = 2;
  
  Serializer s1 = registry.getDataType<int32_t>().createSerializer();
  Serializer s2 = registry.getDataType<double>().createSerializer();
  Serializer s3 = registry.getDataType("int32[3]").createSerializer();
  Serializer s4 = registry.getDataType("int32[]").createSerializer();
  
  s1.serialize(o1, b1);
  s3.serialize(o1, a1);
}
