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

#include "variant_topic_tools/Exceptions.h"
#include "variant_topic_tools/MessageVariable.h"

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

MessageVariable::MessageVariable() {
}

MessageVariable::MessageVariable(const std::string& name, const DataType&
    type) {
  impl.reset(new Impl(name, type));
}

MessageVariable::MessageVariable(const MessageVariable& src) :
  MessageMember(src) {
}

MessageVariable::MessageVariable(const MessageMember& src) :
  MessageMember(src) {
  if (impl)
    BOOST_ASSERT(boost::dynamic_pointer_cast<MessageVariable::Impl>(impl));
}

MessageVariable::~MessageVariable() {
}

MessageVariable::Impl::Impl(const std::string& name, const DataType& type) :
  MessageMember::Impl(name),
  type(type) {
  if (!type.isValid())
    throw InvalidDataTypeException();
}

MessageVariable::Impl::~Impl() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

const DataType& MessageVariable::Impl::getType() const {
  return type;
}

size_t MessageVariable::Impl::getSize() const {
  return type.getSize();
}

bool MessageVariable::Impl::isFixedSize() const {
  return type.isFixedSize();
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void MessageVariable::Impl::write(std::ostream& stream) const {
  stream << type << " " << name;
}

}
