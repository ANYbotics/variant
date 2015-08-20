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
#include "variant_topic_tools/MessageConstant.h"
#include "variant_topic_tools/MessageMember.h"
#include "variant_topic_tools/MessageVariable.h"

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

MessageMember::MessageMember() {
}

MessageMember::MessageMember(const MessageMember& src) :
  impl(src.impl) {
}

MessageMember::~MessageMember() {
}

MessageMember::Impl::Impl(const std::string& name) :
  name(name) {
}

MessageMember::Impl::~Impl() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

const std::string& MessageMember::getName() const {
  if (!impl) {
    static std::string name;
    return name;
  }
  else
    return impl->name;
}

const DataType& MessageMember::getType() const {
  if (!impl) {
    static DataType type;
    return type;
  }
  else
    return impl->getType();
}

size_t MessageMember::getSize() const {
  if (impl)
    return impl->getSize();
  else
    return 0;
}

bool MessageMember::isVariable() const {
  if (impl)
    return boost::dynamic_pointer_cast<MessageVariable::Impl>(impl);
  else
    return false;
}

bool MessageMember::isConstant() const {
  if (impl)
    return boost::dynamic_pointer_cast<MessageConstant::Impl>(impl);
  else
    return false;
}

bool MessageMember::isFixedSize() const {
  if (impl)
    return impl->isFixedSize();
  else
    return false;
}

bool MessageMember::isValid() const {
  return impl;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void MessageMember::write(std::ostream& stream) const {
  if (impl)
    impl->write(stream);
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

std::ostream& operator<<(std::ostream& stream, const MessageMember&
    messageMember) {
  messageMember.write(stream);
  return stream;
}

}
