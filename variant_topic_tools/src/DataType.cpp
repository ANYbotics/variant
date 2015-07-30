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

#include "variant_topic_tools/DataType.h"

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

DataType::DataType() {
}

DataType::DataType(const char* identifier) {
  boost::unordered_map<std::string, DataType>::const_iterator it =
    getInstances().find(identifier);
    
  if (it != getInstances().end())
    impl = it->second.impl;
}

DataType::DataType(const DataType& src) :
  impl(src.impl) {
}

DataType::~DataType() {
}

DataType::Impl::Impl(const std::string& identifier) :
  identifier(identifier) {
}

DataType::Impl::~Impl() {
}

DataType::Instances::Instances() {
  add<bool>("bool");
  add<double>("float64");
  add<float>("float32");
  add<int16_t>("int16");
  add<int32_t>("int32");
  add<int64_t>("int64");
  add<int8_t>("int8");
  add<uint16_t>("uint16");
  add<uint32_t>("uint32");
  add<uint64_t>("uint64");
  add<uint8_t>("uint8");
  
  add<ros::Duration>("duration");
  add<std::string>("string");
  add<ros::Time>("time");
}

DataType::Instances::~Instances() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

std::string DataType::getIdentifier() const {
  if (impl)
    return impl->identifier;
  else
    return std::string();
}

const std::type_info& DataType::getInfo() const {
  if (impl)
    return impl->getInfo();
  else
    return typeid(void);
}

size_t DataType::getSize() const {
  if (impl)
    return impl->getSize();
  else
    return 0;
}

DataType DataType::getMemberType() const {
  if (impl && impl->isArray())
    return impl->memberTypes[0];
  else
    return DataType();
}

std::vector<DataType> DataType::getMemberTypes() const {
  if (impl)
    return impl->memberTypes;
  else
    return std::vector<DataType>();
}

bool DataType::isPrimitive() const {
  if (impl)
    return impl->isPrimitive();
  else
    return false;
}

bool DataType::isSimple() const {
  if (impl)
    return impl->isSimple();
  else
    return false;
}

bool DataType::isFixedSize() const {
  if (impl)
    return impl->isFixedSize();
  else
    return false;
}

bool DataType::isArithmetic() const {
  if (impl)
    return impl->isArithmetic();
  else
    return false;
}

bool DataType::isString() const {
  if (impl)
    return impl->isString();
  else
    return false;
}

bool DataType::isArray() const {
  if (impl)
    return impl->isArray();
  else
    return false;
}

bool DataType::isMessage() const {
  if (impl)
    return impl->isMessage();
  else
    return false;
}

bool DataType::isValid() const {
  if (impl)
    return impl->isValid();
  else
    return false;
}

bool DataType::Impl::isValid() const {
  return !identifier.empty();
}

DataType::Instances& DataType::getInstances() {
  static boost::shared_ptr<Instances> instances(new Instances());
  return *instances;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void DataType::clear() {
  impl.reset();
}

void DataType::write(std::ostream& stream) const {
  if (impl && impl->isValid())
    stream << impl->identifier;
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

std::ostream& operator<<(std::ostream& stream, const DataType& dataType) {
  dataType.write(stream);
}

}
