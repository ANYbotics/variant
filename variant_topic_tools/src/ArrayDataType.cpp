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
#include "variant_topic_tools/ArraySerializer.h"
#include "variant_topic_tools/ArrayVariant.h"
#include "variant_topic_tools/Exceptions.h"

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

ArrayDataType::ArrayDataType() = default;

ArrayDataType::ArrayDataType(const DataType& memberType, size_t numMembers) {
  impl.reset(new boost::shared_ptr<DataType::Impl>(new ImplV(memberType, numMembers)));
}

ArrayDataType::ArrayDataType(const ArrayDataType& src) = default;

ArrayDataType::ArrayDataType(const DataType& src) : DataType(src) {
  if (impl) {
    BOOST_ASSERT(boost::dynamic_pointer_cast<Impl>(*impl));
  }
}

ArrayDataType::~ArrayDataType() = default;

ArrayDataType::Impl::Impl(const DataType& memberType) : memberType(memberType) {
  if (!memberType.isValid()) {
    throw InvalidDataTypeException();
  }
}

ArrayDataType::Impl::~Impl() = default;

ArrayDataType::ImplV::ImplV(const DataType& memberType, size_t numMembers) : Impl(memberType), numMembers(numMembers) {}

ArrayDataType::ImplV::~ImplV() = default;

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

const DataType& ArrayDataType::getMemberType() const {
  if (!impl) {
    static DataType memberType;
    return memberType;
  } else {
    return boost::static_pointer_cast<Impl>(*impl)->memberType;
  }
}

size_t ArrayDataType::getNumMembers() const {
  if (impl) {
    return boost::static_pointer_cast<Impl>(*impl)->getNumMembers();
  } else {
    return 0;
  }
}

bool ArrayDataType::isDynamic() const {
  if (impl) {
    return boost::static_pointer_cast<Impl>(*impl)->isDynamic();
  } else {
    return 0;
  }
}

const std::string& ArrayDataType::Impl::getIdentifier() const {
  if (identifier.empty()) {
    identifier = memberType.getIdentifier() + ((getNumMembers() != 0u) ? "[" + std::to_string(getNumMembers()) + "]" : "[]");
  }

  return identifier;
}

size_t ArrayDataType::ImplV::getNumMembers() const {
  return numMembers;
}

size_t ArrayDataType::ImplV::getSize() const {
  return numMembers * memberType.getSize();
}

bool ArrayDataType::ImplV::isDynamic() const {
  return numMembers == 0u;
}

bool ArrayDataType::ImplV::isFixedSize() const {
  return (numMembers != 0u) && memberType.isFixedSize();
}

bool ArrayDataType::ImplV::isSimple() const {
  return false;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

Serializer ArrayDataType::ImplV::createSerializer(const DataType& /*type*/) const {
  return ArraySerializer(memberType.createSerializer(), numMembers);
}

Variant ArrayDataType::ImplV::createVariant(const DataType& type) const {
  return ArrayVariant(type, memberType, numMembers);
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

ArrayDataType& ArrayDataType::operator=(const DataType& src) {
  DataType::operator=(src);

  if (impl) {
    BOOST_ASSERT(boost::dynamic_pointer_cast<ArrayDataType::Impl>(*impl));
  }

  return *this;
}

}  // namespace variant_topic_tools
