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

#include "variant_topic_tools/MessageSerializer.h"

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

MessageSerializer::MessageSerializer() {
}

MessageSerializer::MessageSerializer(const DataType& dataType) {
  boost::unordered_map<DataType, MessageSerializer>::const_iterator
    it = getInstances().find(dataType);
    
  if (it != getInstances().end())
    impl = it->second.impl;
}

MessageSerializer::MessageSerializer(const MessageSerializer& src) :
  impl(src.impl) {
}

MessageSerializer::~MessageSerializer() {
}

MessageSerializer::Impl::Impl(const DataType& dataType) :
  dataType(dataType) {
}

MessageSerializer::Impl::~Impl() {
}

MessageSerializer::Instances::Instances() {
  createSimple<bool>("bool");
  createSimple<double>("float64");
  createSimple<float>("float32");
  createSimple<int16_t>("int16");
  createSimple<int32_t>("int32");
  createSimple<int64_t>("int64");
  createSimple<int8_t>("int8");
  createSimple<uint16_t>("uint16");
  createSimple<uint32_t>("uint32");
  createSimple<uint64_t>("uint64");
  createSimple<uint8_t>("uint8");
  
//   createBuiltin<ros::Duration>("duration");
//   createBuiltin<std::string>("string");
//   createBuiltin<ros::Time>("time");
}

MessageSerializer::Instances::~Instances() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

DataType MessageSerializer::getDataType() const {
  if (impl)
    return impl->dataType;
  else
    return DataType();
}

bool MessageSerializer::Impl::isValid() const {
  return dataType;
}

MessageSerializer::Instances& MessageSerializer::getInstances() {
  static boost::shared_ptr<Instances> instances(new Instances());
  return *instances;
}

}
