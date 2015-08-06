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

#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <variant_topic_tools/DataTypeRegistry.h>

int main(int argc, char **argv) {
  variant_topic_tools::DataTypeRegistry registry;

  registry.addArrayDataType<unsigned char[3]>();
  registry.addArrayDataType<unsigned char*>();
  registry.addArrayDataType<int, 3>();
  registry.addArrayDataType<int, 0>();
  registry.addArrayDataType<boost::array<double, 3> >();
  registry.addArrayDataType<std::vector<double> >();
  registry.addArrayDataType("int8", 3);
  registry.addArrayDataType(typeid(int8_t));
  
  registry.addMessageDataType<std_msgs::String>();
  registry.addMessageDataType(ros::message_traits::datatype<std_msgs::Bool>(),
    ros::message_traits::definition<std_msgs::Bool>());
  registry.addMessageDataType("my_msgs/Double").addVariable<double>("data");
  registry.addMessageDataType("my_msgs/Complex",
    "float64 PI=3.14159\nstring COMMENT=This is a complex message type.\n"
    "float64 real\nfloat64 imaginary\n");
  
  std::cout << registry << "\n";
  std::cout << "\n";

  std::cout << registry.getDataType<int8_t>() << "\n";
  std::cout << registry.getDataType<uint8_t>() << "\n";
  std::cout << "\n";
  
  variant_topic_tools::Variant v1 = registry["float64"].createVariant();
  std::cout << v1 << "\n";
  v1 = 3.14159;
  std::cout << v1 << "\n";
  std::cout << "\n";
  
  variant_topic_tools::Variant v2 = registry["float64[3]"].createVariant();
  std::cout << v2.getType().getIdentifier() << "\n";
  std::cout << v2.getValue<boost::array<double, 3> >()[0] << "\n";
  v2.getValue<boost::array<double, 3> >()[1] = 1.0;
  std::cout << v2.getValue<boost::array<double, 3> >()[1] << "\n";
  std::cout << "\n";
  
  variant_topic_tools::Variant v3 = registry["float64[]"].createVariant();
  std::cout << v3.getType().getIdentifier() << "\n";
  std::cout << v3.getValue<std::vector<double> >().size() << "\n";
  v3.getValue<std::vector<double> >().push_back(0.0);
  std::cout << v3.getValue<std::vector<double> >().size() << "\n";
  std::cout << "\n";
  
  variant_topic_tools::Variant v4 = registry["int8[3]"].createVariant();
  std::cout << v4.isEmpty() << "\n";
  std::cout << "\n";
  
  variant_topic_tools::ArrayDataType a1 = registry["uint8[3]"];
  std::cout << a1.getNumElements() << "\n";
  std::cout << "\n";
  
  variant_topic_tools::MessageDataType m1 = registry["std_msgs/String"];
  std::cout << m1.getMD5Sum() << "\n";
  std::cout << m1.getDefinition();
  std::cout << m1.getSize() << "\n";
  std::cout << m1.getNumMembers() << "\n";
  std::cout << "\n";
  
  variant_topic_tools::MessageDataType m2 = registry["std_msgs/Bool"];
  std::cout << m2.getMD5Sum() << "\n";
  std::cout << m2.getDefinition();
  std::cout << m2.getSize() << "\n";
  std::cout << m2.getNumMembers() << "\n";
  std::cout << "\n";
  
  variant_topic_tools::MessageDataType m3 = registry["my_msgs/Double"];
  m3.addConstant("PI", M_PI);
  std::cout << m3.getMD5Sum() << "\n";
  std::cout << m3.getDefinition();
  std::cout << m3.getSize() << "\n";
  std::cout << m3.getNumMembers() << "\n";
  std::cout << "\n";
  
  variant_topic_tools::MessageDataType m4 = registry["my_msgs/Complex"];
  std::cout << m4.getMD5Sum() << "\n";
  std::cout << m4.getDefinition();
  std::cout << m4.getSize() << "\n";
  std::cout << m4.getNumMembers() << "\n";
  std::cout << m4[0] << "\n";
  std::cout << m4[1] << "\n";
  std::cout << "\n";
  
  return 0;
}
