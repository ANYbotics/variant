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

#include "variant_topic_tools/ArrayDataType.h"
#include "variant_topic_tools/BuiltinDataType.h"
#include "variant_topic_tools/DataType.h"
#include "variant_topic_tools/DataTypeRegistry.h"
#include "variant_topic_tools/MessageDataType.h"
#include <variant_topic_tools/Serializer.h>
#include "variant_topic_tools/Variant.h"

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

DataType::DataType() {
}

DataType::DataType(const char* identifier) {
  DataTypeRegistry registry;
  DataType dataType = registry.getDataType(identifier);
  
  impl = dataType.impl;
}

DataType::DataType(const std::string& identifier) {
  DataTypeRegistry registry;
  DataType dataType = registry.getDataType(identifier);
  
  impl = dataType.impl;
}

DataType::DataType(const std::type_info& typeInfo) {
  DataTypeRegistry registry;
  DataType dataType = registry.getDataType(typeInfo);
  
  impl = dataType.impl;
}

DataType::DataType(const DataType& src) {
  if (src.impl)
    impl.reset(new boost::shared_ptr<Impl>(*src.impl));
}

DataType::~DataType() {
}

DataType::Impl::Impl() {
}

DataType::Impl::~Impl() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

const std::string& DataType::getIdentifier() const {
  if (!impl) {
    static std::string identifier;
    return identifier;
  }
  else
    return (*impl)->getIdentifier();
}

const std::type_info& DataType::getTypeInfo() const {
  if (impl)
    return (*impl)->getTypeInfo();
  else
    return typeid(void);
}

size_t DataType::getSize() const {
  if (impl)
    return (*impl)->getSize();
  else
    return 0;
}

bool DataType::isArray() const {
  if (impl)
    return boost::dynamic_pointer_cast<ArrayDataType::Impl>(*impl) != 0;
  else
    return false;
}

bool DataType::isBuiltin() const {
  if (impl)
    return boost::dynamic_pointer_cast<BuiltinDataType::Impl>(*impl) != 0;
  else
    return false;
}

bool DataType::isMessage() const {
  if (impl)
    return boost::dynamic_pointer_cast<MessageDataType::Impl>(*impl) != 0;
  else
    return false;
}

bool DataType::isFixedSize() const {
  if (impl)
    return (*impl)->isFixedSize();
  else
    return true;
}

bool DataType::isSimple() const {
  if (impl)
    return (*impl)->isSimple();
  else
    return true;
}

bool DataType::isValid() const {
  return impl != 0;
}

bool DataType::hasTypeInfo() const {
  if (impl)
    return ((*impl)->getTypeInfo() != typeid(void));
  else
    return false;
}

const std::type_info& DataType::Impl::getTypeInfo() const {
  return typeid(void);
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void DataType::clear() {
  impl.reset();
}

void DataType::write(std::ostream& stream) const {
  if (impl)
    stream << (*impl)->getIdentifier();
}

Serializer DataType::createSerializer() const {
  if (impl)
    return (*impl)->createSerializer(*this);
  else
    return Serializer();
}

Variant DataType::createVariant() const {
  if (impl)
    return (*impl)->createVariant(*this);
  else
    return Variant();
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

DataType& DataType::operator=(const DataType& src) {
  if (impl && src.impl)
    *impl = *src.impl;
  else if (src.impl)
    impl.reset(new boost::shared_ptr<Impl>(*src.impl));
  else
    impl = src.impl;
  
  return *this;
}

std::ostream& operator<<(std::ostream& stream, const DataType& dataType) {
  dataType.write(stream);
  return stream;
}

}
