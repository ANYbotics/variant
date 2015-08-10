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
#include "variant_topic_tools/Exceptions.h"

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

ArrayDataType::ArrayDataType() {
}

ArrayDataType::ArrayDataType(const DataType& elementType, size_t numElements) {
  impl.reset(new DataType::ImplA(new ImplV(elementType, numElements)));
}

ArrayDataType::ArrayDataType(const ArrayDataType& src) :
  DataType(src) {
}

ArrayDataType::ArrayDataType(const DataType& src) :
  DataType(src) {
  if (impl)
    BOOST_ASSERT(boost::dynamic_pointer_cast<ArrayDataType::Impl>(
      impl->adaptee));
}

ArrayDataType::~ArrayDataType() {
}

ArrayDataType::Impl::Impl(const DataType& elementType) :
  elementType(elementType) {
  if (!elementType.isValid())
    throw InvalidDataTypeException();  
}

ArrayDataType::Impl::~Impl() {
}

ArrayDataType::ImplV::ImplV(const DataType& elementType, size_t numElements) :
  Impl(elementType),
  numElements(numElements) {
}

ArrayDataType::ImplV::~ImplV() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

const DataType& ArrayDataType::getElementType() const {
  if (!impl) {
    static DataType elementType;
    return elementType;
  }
  else
    return boost::dynamic_pointer_cast<Impl>(impl->adaptee)->elementType;
}

size_t ArrayDataType::getNumElements() const {
  if (impl)
    return boost::dynamic_pointer_cast<Impl>(impl->adaptee)->getNumElements();
  else
    return 0;
}

const std::string& ArrayDataType::Impl::getIdentifier() const {
  if (identifier.empty()) {
    identifier = elementType.getIdentifier()+(isFixedSize() ?
      "["+boost::lexical_cast<std::string>(getNumElements())+"]" : "[]");
  }

  return identifier;
}

size_t ArrayDataType::Impl::getSize() const {
  return getNumElements()*elementType.getSize();
}

bool ArrayDataType::Impl::isFixedSize() const {
  return getNumElements();
}

size_t ArrayDataType::ImplV::getNumElements() const {
  return numElements;
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

ArrayDataType& ArrayDataType::operator=(const DataType& src) {
  DataType::operator=(src);
  
  if (impl)
    BOOST_ASSERT(boost::dynamic_pointer_cast<ArrayDataType::Impl>(
      impl->adaptee));
    
  return *this;
}

}
