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

#include "variant_topic_tools/BuiltinDataType.h"

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

BuiltinDataType::BuiltinDataType() {
}

BuiltinDataType::BuiltinDataType(const BuiltinDataType& src) :
  DataType(src) {
}

BuiltinDataType::BuiltinDataType(const DataType& src) :
  DataType(src) {
  if (impl)
    BOOST_ASSERT(boost::dynamic_pointer_cast<BuiltinDataType::Impl>(
      impl->adaptee));
}

BuiltinDataType::~BuiltinDataType() {
}

BuiltinDataType::Impl::Impl(const std::string& identifier) :
  identifier(identifier) {
}

BuiltinDataType::Impl::~Impl() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

const std::string& BuiltinDataType::Impl::getIdentifier() const {
  return identifier;
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

BuiltinDataType& BuiltinDataType::operator=(const DataType& src) {
  DataType::operator=(src);
  
  if (impl)
    BOOST_ASSERT(boost::dynamic_pointer_cast<BuiltinDataType::Impl>(
      impl->adaptee));
    
  return *this;
}

}
