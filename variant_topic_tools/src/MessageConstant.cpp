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

#include <sstream>

#include "variant_topic_tools/Exceptions.h"
#include "variant_topic_tools/MessageConstant.h"

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

MessageConstant::MessageConstant() = default;

MessageConstant::MessageConstant(const std::string& name, const Variant& value) {
  impl.reset(new Impl(name, value));
}

MessageConstant::MessageConstant(const std::string& name, const DataType& type, const std::string& value) {
  Variant variantValue = type.createVariant();

  if (!variantValue.isEmpty()) {
    if (type.getTypeInfo() != typeid(std::string)) {
      std::istringstream stream(value);
      stream >> variantValue;
    } else {
      variantValue = value;
    }

    impl.reset(new Impl(name, variantValue));
  }
}

MessageConstant::MessageConstant(const MessageConstant& src) = default;

MessageConstant::MessageConstant(const MessageMember& src) : MessageMember(src) {
  if (impl) {
    BOOST_ASSERT(boost::dynamic_pointer_cast<MessageConstant::Impl>(impl));
  }
}

MessageConstant::~MessageConstant() = default;

MessageConstant::Impl::Impl(const std::string& name, const Variant& value) : MessageMember::Impl(name), value(value) {
  if (!value.getType().isValid()) {
    throw InvalidDataTypeException();
  }
}

MessageConstant::Impl::~Impl() = default;

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

const Variant& MessageConstant::getValue() const {
  if (!impl) {
    static Variant value;
    return value;
  } else {
    return boost::static_pointer_cast<Impl>(impl)->value;
  }
}

const DataType& MessageConstant::Impl::getType() const {
  return value.getType();
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void MessageConstant::Impl::write(std::ostream& stream) const {
  stream << "const " << value.getType() << " " << name << "=" << value;
}

}  // namespace variant_topic_tools
